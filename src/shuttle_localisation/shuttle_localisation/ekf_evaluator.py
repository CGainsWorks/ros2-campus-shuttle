#!/usr/bin/env python3
"""Compare the EKF estimate against Gazebo ground truth.

The brief asks for the EKF to be "validated against ground truth". This node is
that validation: it subscribes to the filtered pose and to the p3d ground-truth
pose, aligns them in time, and reports position and heading error, plus whether
the reported covariance is consistent with the error actually observed.

The consistency check matters more than the raw error. A filter that is
confidently wrong is more dangerous than one that is honestly uncertain, and
the normalised estimation error squared (NEES) is what exposes that.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class EkfEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("ekf_evaluator")

        self.declare_parameter("output_csv", "results/gazebo/ekf_evaluation.csv")
        self.declare_parameter("truth_topic", "/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/odometry/filtered")
        self.declare_parameter("max_pair_dt", 0.05)

        self.output_csv = self.get_parameter("output_csv").value
        self.max_pair_dt = float(self.get_parameter("max_pair_dt").value)

        # Ground truth arrives at 50 Hz and the filter at 50 Hz, but not in
        # lockstep. Buffer truth and match each estimate to its nearest sample
        # rather than assuming the two streams are aligned.
        self.truth = deque(maxlen=400)
        self.rows: list[dict] = []

        self.create_subscription(
            Odometry, self.get_parameter("truth_topic").value, self.on_truth, 50)
        self.create_subscription(
            Odometry, self.get_parameter("estimate_topic").value, self.on_estimate, 50)

        self.get_logger().info("EKF evaluator running")

    def on_truth(self, msg: Odometry) -> None:
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self.truth.append((
            t,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        ))

    def nearest_truth(self, t: float):
        if not self.truth:
            return None
        best = min(self.truth, key=lambda s: abs(s[0] - t))
        if abs(best[0] - t) > self.max_pair_dt:
            return None
        return best

    def on_estimate(self, msg: Odometry) -> None:
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        truth = self.nearest_truth(t)
        if truth is None:
            return

        _, tx, ty, tyaw = truth
        ex = msg.pose.pose.position.x
        ey = msg.pose.pose.position.y
        eyaw = yaw_from_quaternion(msg.pose.pose.orientation)

        dx = ex - tx
        dy = ey - ty
        dyaw = wrap_to_pi(eyaw - tyaw)

        cov = msg.pose.covariance
        var_x = max(cov[0], 1e-9)
        var_y = max(cov[7], 1e-9)
        var_yaw = max(cov[35], 1e-9)

        # NEES for the (x, y, yaw) sub-state, treating the axes as independent.
        # For 3 degrees of freedom the expected value is 3.0 if the filter's
        # covariance is honest.
        nees = dx * dx / var_x + dy * dy / var_y + dyaw * dyaw / var_yaw

        self.rows.append({
            "t": t,
            "truth_x": tx, "truth_y": ty, "truth_yaw": tyaw,
            "est_x": ex, "est_y": ey, "est_yaw": eyaw,
            "err_x": dx, "err_y": dy,
            "err_pos": math.hypot(dx, dy),
            "err_yaw": dyaw,
            "std_x": math.sqrt(var_x),
            "std_y": math.sqrt(var_y),
            "std_yaw": math.sqrt(var_yaw),
            "nees": nees,
        })

    def report(self) -> None:
        if not self.rows:
            self.get_logger().warn("no paired samples, nothing to report")
            return

        os.makedirs(os.path.dirname(self.output_csv) or ".", exist_ok=True)
        with open(self.output_csv, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

        pos = [r["err_pos"] for r in self.rows]
        yaw = [abs(r["err_yaw"]) for r in self.rows]
        nees = [r["nees"] for r in self.rows]
        n = len(pos)

        def rms(values):
            return math.sqrt(sum(v * v for v in values) / len(values))

        mean_nees = sum(nees) / n
        self.get_logger().info(
            f"EKF vs ground truth over {n} samples:\n"
            f"  position error : RMS {rms(pos):.4f} m, max {max(pos):.4f} m\n"
            f"  heading error  : RMS {rms(yaw):.4f} rad, max {max(yaw):.4f} rad\n"
            f"  mean NEES      : {mean_nees:.2f} (3.0 = covariance consistent; "
            f"much higher means the filter is overconfident)\n"
            f"  wrote {self.output_csv}"
        )


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = EkfEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
