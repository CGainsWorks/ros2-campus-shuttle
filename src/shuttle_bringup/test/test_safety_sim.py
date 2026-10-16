"""The safety gate stops the simulated base: e-stop and command timeout.

Starts the Gazebo simulation (empty world) with the safety gate in front of the
differential drive, drives the robot at 1.0 m/s through /cmd_vel_in and then
stops it two ways:

* **e-stop**: the `set_e_stop` service is called while commands keep arriving
  at 20 Hz. The base must come to rest within ESTOP_BOUND seconds and stay
  there, because a latched stop drops every command.
* **command timeout**: commands simply stop. The base must come to rest within
  WATCHDOG_BOUND seconds of the last command (the gate's 0.5 s timeout plus
  braking) and must still have been moving shortly before the timeout, which
  shows it is the watchdog that stopped it.

"At rest" is a ground-truth speed below STOP_SPEED, taken from the physics
engine rather than from odometry and every time is simulated time, which is
the time the robot experiences. The measured figures are printed and, when
SHUTTLE_TEST_OUTPUT_DIR is set, written to safety_stop_times.json.

    launch_test src/shuttle_bringup/test/test_safety_sim.py
"""

import json
import math
import os
import time
import unittest

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

CMD_TIMEOUT = 0.5      # s, the gate's command timeout for this test
DRIVE_SPEED = 1.0      # m/s
STOP_SPEED = 0.05      # m/s, ground-truth speed counted as "at rest"
ESTOP_BOUND = 0.5      # s from the e-stop call to rest
WATCHDOG_BOUND = 1.0   # s from the last command to rest (timeout + braking)
STARTUP_TIMEOUT = 240  # s of wall time for the simulator and robot to come up


@pytest.mark.launch_test
def generate_test_description():
    bringup = FindPackageShare("shuttle_bringup")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "sim.launch.py"])),
        launch_arguments={
            "world": PathJoinSubstitution([bringup, "worlds", "empty_test.sdf"]),
            "headless": "true",
            "rviz": "false",
            "cmd_timeout": str(CMD_TIMEOUT),
        }.items(),
    )
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def time_to_rest(samples, t0):
    """Seconds from t0 until the speed drops below STOP_SPEED for good.

    Returns None if the robot is not at rest by the last sample.
    """
    after = [(t, v) for t, v in samples if t >= t0]
    if not after or after[-1][1] >= STOP_SPEED:
        return None
    rest = after[-1][0]
    for t, v in reversed(after):
        if v >= STOP_SPEED:
            break
        rest = t
    return rest - t0


class TestSafetyLayer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            "safety_layer_test",
            parameter_overrides=[Parameter("use_sim_time", value=True)])
        cls.truth = []  # (sim time, ground-truth speed)
        cls.estop_active = None
        cls.node.create_subscription(
            Odometry, "/ground_truth/odom", cls._on_truth, 50)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(Bool, "/e_stop/active", cls._on_state, latched)
        cls.cmd_pub = cls.node.create_publisher(Twist, "/cmd_vel_in", 10)
        cls.estop_client = cls.node.create_client(SetBool, "/set_e_stop")
        cls.results = {}

    @classmethod
    def tearDownClass(cls):
        out_dir = os.environ.get("SHUTTLE_TEST_OUTPUT_DIR")
        if out_dir and cls.results:
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, "safety_stop_times.json")
            with open(path, "w", encoding="utf-8", newline="\n") as fh:
                json.dump(cls.results, fh, indent=2, sort_keys=True)
                fh.write("\n")
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_truth(cls, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        cls.truth.append((t, v))

    @classmethod
    def _on_state(cls, msg):
        cls.estop_active = msg.data

    # -- helpers -----------------------------------------------------------

    def sim_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def speed(self):
        return self.truth[-1][1] if self.truth else 0.0

    def spin_for(self, wall_seconds):
        end = time.monotonic() + wall_seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def spin_until(self, predicate, wall_timeout, what):
        end = time.monotonic() + wall_timeout
        while time.monotonic() < end:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail(f"timed out after {wall_timeout} s waiting for {what}")

    def drive(self, sim_seconds, wall_timeout=120.0, stop_when=None):
        """Publish the drive command at 20 Hz for sim_seconds of sim time."""
        cmd = Twist()
        cmd.linear.x = DRIVE_SPEED
        start = self.sim_now()
        end_wall = time.monotonic() + wall_timeout
        next_pub = 0.0
        last_pub_sim = start
        while self.sim_now() - start < sim_seconds:
            if time.monotonic() > end_wall:
                self.fail("simulation stopped advancing while driving")
            if time.monotonic() >= next_pub:
                self.cmd_pub.publish(cmd)
                last_pub_sim = self.sim_now()
                next_pub = time.monotonic() + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if stop_when is not None and stop_when():
                break
        return last_pub_sim

    def wait_sim(self, sim_seconds, wall_timeout=120.0):
        start = self.sim_now()
        self.spin_until(lambda: self.sim_now() - start >= sim_seconds,
                        wall_timeout, f"{sim_seconds} s of simulated time")

    def set_estop(self, engage):
        self.assertTrue(self.estop_client.wait_for_service(timeout_sec=30.0),
                        "set_e_stop service not available")
        future = self.estop_client.call_async(SetBool.Request(data=engage))
        self.spin_until(future.done, 30.0, "the set_e_stop reply")
        self.assertTrue(future.result().success)

    def come_up_to_speed(self):
        self.spin_until(lambda: len(self.truth) > 10, STARTUP_TIMEOUT,
                        "ground truth from the spawned robot")
        self.spin_until(lambda: self.estop_active is not None, 60.0,
                        "the safety gate's e-stop state")
        self.drive(10.0, stop_when=lambda: self.speed() > 0.95 * DRIVE_SPEED)
        self.assertGreater(self.speed(), 0.95 * DRIVE_SPEED,
                           "the base never reached the drive speed through the gate")
        self.drive(1.0)  # a little steady running at speed

    # -- tests -------------------------------------------------------------

    def test_1_estop_stops_the_base(self):
        self.set_estop(False)
        self.come_up_to_speed()
        speed_before = self.speed()

        t_stop = self.sim_now()
        self.set_estop(True)
        # Keep commanding full speed: a latched stop must ignore it.
        self.drive(3.0)
        self.wait_sim(0.5)

        self.assertTrue(self.estop_active, "e_stop/active did not report the stop")
        stop_time = time_to_rest(self.truth, t_stop)
        self.assertIsNotNone(
            stop_time, "the base was still moving 3 s after the e-stop, "
            "with commands still arriving")
        print(f"\ne-stop: {speed_before:.2f} m/s to rest in {stop_time:.3f} s "
              f"of simulated time (bound {ESTOP_BOUND} s)")
        self.results["estop"] = {
            "initial_speed_mps": round(speed_before, 3),
            "time_to_rest_s": round(stop_time, 3),
            "bound_s": ESTOP_BOUND,
        }
        self.assertLessEqual(stop_time, ESTOP_BOUND)

        # Releasing must not resume the commands sent during the stop.
        self.set_estop(False)
        self.wait_sim(1.0)
        self.assertLess(self.speed(), STOP_SPEED,
                        "the base moved after the release without a fresh command")
        self.assertFalse(self.estop_active)

    def test_2_command_timeout_stops_the_base(self):
        self.set_estop(False)
        self.come_up_to_speed()
        speed_before = self.speed()

        t_last = self.drive(0.2)  # the final command
        self.wait_sim(3.0)

        # Still moving partway into the timeout window, so it is the watchdog
        # and not something else that stopped it.
        moving = [v for t, v in self.truth if t_last < t <= t_last + 0.6 * CMD_TIMEOUT]
        self.assertTrue(moving and min(moving) > 0.5 * DRIVE_SPEED,
                        "the base slowed before the command timeout expired")

        stop_time = time_to_rest(self.truth, t_last)
        self.assertIsNotNone(stop_time, "the base never stopped after commands ceased")
        print(f"\ncommand timeout: {speed_before:.2f} m/s to rest {stop_time:.3f} s "
              f"after the last command (timeout {CMD_TIMEOUT} s, bound "
              f"{WATCHDOG_BOUND} s)")
        self.results["command_timeout"] = {
            "initial_speed_mps": round(speed_before, 3),
            "cmd_timeout_s": CMD_TIMEOUT,
            "time_to_rest_s": round(stop_time, 3),
            "bound_s": WATCHDOG_BOUND,
        }
        self.assertLessEqual(stop_time, WATCHDOG_BOUND)
