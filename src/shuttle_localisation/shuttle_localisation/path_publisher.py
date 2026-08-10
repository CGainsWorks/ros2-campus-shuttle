#!/usr/bin/env python3
"""Publish a fixed reference path for the controller comparison.

The comparison must replay *identical* trajectories through both controllers.
Asking Nav2 to plan the route each time would not guarantee that: the planner
can return a slightly different path between runs, and any difference in the
reference would confound the difference between the controllers.

So the scenario geometry is generated from scenario_paths.py -- the same
geometry the offline benchmark uses, checked against the C++ implementation by
tests/test_scenarios_match.py -- and published latched on /plan, so both
controllers receive a byte-identical sequence of poses.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from shuttle_localisation.scenario_paths import SCENARIOS


class PathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("path_publisher")
        self.declare_parameter("scenario", "campus_loop")
        self.declare_parameter("frame_id", "map")

        scenario = self.get_parameter("scenario").value
        if scenario not in SCENARIOS:
            raise ValueError(
                f"unknown scenario '{scenario}', expected one of "
                f"{sorted(SCENARIOS)}")

        points, start = SCENARIOS[scenario]()
        self.get_logger().info(
            f"scenario '{scenario}': {len(points)} poses, "
            f"start ({start[0]:.2f}, {start[1]:.2f}, "
            f"{math.degrees(start[2]):.0f} deg)")

        # Latched, so a controller that starts late still receives the plan.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Path, "plan", qos)
        self.msg = self.build_path(points)
        self.timer = self.create_timer(1.0, self.publish)
        self.publish()

    def build_path(self, points) -> Path:
        msg = Path()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            nxt = points[min(i + 1, len(points) - 1)]
            yaw = math.atan2(nxt[1] - y, nxt[0] - x) if nxt != (x, y) else 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        return msg

    def publish(self) -> None:
        self.pub.publish(self.msg)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
