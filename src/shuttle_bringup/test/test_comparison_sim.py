"""Headless integration test: simulator, stack and comparison scenario.

For each controller, starts comparison.launch.py exactly as
scripts/run_comparison.sh does (Gazebo, the bridge, the safety gate, the local
EKF, the path publisher, the path tracker, the metrics recorder and the EKF
evaluator) on the `corner` route at 1.0 m/s and asserts that

* the robot reaches the goal (the tracker reports it and the recorder's
  ground-truth progress covers the route to within dead-reckoning drift),
* the controller metrics are produced: a ControllerMetrics message on
  /controller_metrics, a row in metrics.csv and a time series file and
* the tracking error is that of a robot following the route rather than one
  that wandered off it.

The tracking-error bound is a sanity limit, not a result. The numbers this run
produces are written to SHUTTLE_TEST_OUTPUT_DIR (or a temporary directory) so
CI can keep them but the committed Gazebo figures come from the repeated runs
of scripts/run_gazebo_repeats.sh.

    launch_test src/shuttle_bringup/test/test_comparison_sim.py
"""

import csv
import os
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shuttle_msgs.msg import ControllerMetrics

SCENARIO = "corner"
SPEED = "1.0"
START = {"x": "0.0", "y": "0.0", "yaw": "0.0"}  # corner's start pose
RUN_TIMEOUT = 900       # s of wall time for one run, startup included
# Fraction of the route covered, from ground truth. Not 1.0: the tracker stops
# within its 0.25 m goal tolerance of where it BELIEVES the end is and it
# believes that from dead-reckoned odometry, which drifts along the track by a
# few tenths of a metre over this route. 0.95 leaves 1.1 m for the two together.
MIN_PROGRESS = 0.95
MAX_RMS_CROSS_TRACK = 0.30  # m; sanity limit for "followed the route"

OUTPUT_DIR = os.path.join(
    os.environ.get("SHUTTLE_TEST_OUTPUT_DIR") or tempfile.mkdtemp(prefix="shuttle_"),
    "comparison")


@pytest.mark.launch_test
@launch_testing.parametrize("controller", ["pure_pursuit", "stanley"])
def generate_test_description(controller):
    bringup = FindPackageShare("shuttle_bringup")
    comparison = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "comparison.launch.py"])),
        launch_arguments={
            "controller": controller,
            "scenario": SCENARIO,
            "speed": SPEED,
            "output_dir": OUTPUT_DIR,
            "exit_on_finish": "false",
            **START,
        }.items(),
    )
    return LaunchDescription([comparison, launch_testing.actions.ReadyToTest()])


class TestComparisonRun(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("comparison_integration_test")
        self.metrics = []
        # Transient local, so a summary published just before this subscriber
        # matched is still delivered.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            ControllerMetrics, "/controller_metrics", self.metrics.append, qos)

    def tearDown(self):
        self.node.destroy_node()

    def test_run_reaches_goal_and_produces_metrics(self, controller):
        end = time.monotonic() + RUN_TIMEOUT
        while time.monotonic() < end and not self.metrics:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertTrue(
            self.metrics,
            f"{controller}: no ControllerMetrics within {RUN_TIMEOUT} s. The "
            "recorder publishes only once the tracker reports the goal, so "
            "either the robot never got there or no ground truth arrived.")
        m = self.metrics[-1]

        print(f"\n{controller} on {SCENARIO} at {SPEED} m/s (simulated): "
              f"RMS cross-track {m.ct_rms:.4f} m, max {m.ct_max_abs:.4f} m, "
              f"progress {100.0 * m.progress:.1f}%, {m.duration:.1f} s, "
              f"{m.samples} samples")

        self.assertEqual(m.controller, controller)
        self.assertEqual(m.scenario, SCENARIO)
        self.assertTrue(m.goal_reached, "the tracker never reported the goal")
        self.assertGreaterEqual(m.progress, MIN_PROGRESS)
        self.assertGreater(m.samples, 500)
        self.assertLess(m.ct_rms, MAX_RMS_CROSS_TRACK)
        self.assertGreater(m.mean_speed, 0.5)

        # The same numbers on disk, in the format analysis/compare_controllers.py
        # reads.
        metrics_csv = os.path.join(OUTPUT_DIR, "metrics.csv")
        self.assertTrue(os.path.isfile(metrics_csv), f"{metrics_csv} not written")
        with open(metrics_csv, encoding="utf-8") as fh:
            rows = [r for r in csv.DictReader(fh) if r["controller"] == controller]
        self.assertTrue(rows, f"no {controller} row in {metrics_csv}")
        self.assertEqual(rows[-1]["goal_reached"], "1")
        self.assertAlmostEqual(float(rows[-1]["ct_rms"]), m.ct_rms, places=6)

        series = os.path.join(
            OUTPUT_DIR, "timeseries", f"{controller}_{SCENARIO}_gazebo.csv")
        self.assertTrue(os.path.isfile(series), f"{series} not written")
        with open(series, encoding="utf-8") as fh:
            self.assertGreater(sum(1 for _ in fh), 500)
