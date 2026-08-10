#!/usr/bin/env python3
"""Capture camera-to-LiDAR calibration observations from live topics.

Produces the same JSON schema that synth_calib_data.py generates, so the same
solver runs on simulated and real data.

For each capture:
  * the checkerboard is found in the image and solvePnP with the intrinsics
    from /camera/camera_info gives the board pose in the camera frame. The
    board plane is then its z axis and the offset along it;
  * the LiDAR points that fall on the board are isolated by gating on range and
    bearing and fitting a line with RANSAC. A 2D LiDAR sees the board as a line
    segment rather than a patch, but those points still lie on the board plane,
    which is all the point-to-plane residual needs.

Capture is triggered by a service call rather than a keypress, so it works when
launched rather than run from a terminal:

    ros2 service call /collect_calib_data/capture std_srvs/srv/Trigger
    ros2 service call /collect_calib_data/save std_srvs/srv/Trigger

Move the board between captures, and vary its *orientation*, not just its
distance: board poses whose normals are all parallel leave rotation about that
normal unobservable. The solver reports the condition number so a poor set is
visible rather than silently wrong.
"""

from __future__ import annotations

import json
import math
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_srvs.srv import Trigger


class CalibCollector(Node):
    def __init__(self) -> None:
        super().__init__("collect_calib_data")

        self.declare_parameter("output", "calibration/observations.json")
        self.declare_parameter("board_cols", 9)
        self.declare_parameter("board_rows", 6)
        self.declare_parameter("square_size", 0.045)
        self.declare_parameter("scan_max_range", 4.0)
        self.declare_parameter("scan_half_fov", 0.6)
        self.declare_parameter("ransac_threshold", 0.02)
        self.declare_parameter("min_board_points", 8)

        self.output = self.get_parameter("output").value
        self.cols = int(self.get_parameter("board_cols").value)
        self.rows = int(self.get_parameter("board_rows").value)
        self.square = float(self.get_parameter("square_size").value)

        self.bridge = CvBridge()
        self.image = None
        self.camera_info = None
        self.scan = None
        self.observations: list[dict] = []

        # Board corners in the board frame: z = 0 plane.
        objp = np.zeros((self.rows * self.cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.object_points = objp * self.square

        self.create_subscription(Image, "/camera/image_raw", self.on_image, 5)
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_info, 5)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 5)

        self.create_service(Trigger, "~/capture", self.on_capture)
        self.create_service(Trigger, "~/save", self.on_save)

        self.get_logger().info(
            f"ready: {self.cols}x{self.rows} board, {self.square * 1000:.0f} mm "
            f"squares. Call ~/capture for each board pose, then ~/save.")

    # -- subscriptions ----------------------------------------------------

    def on_image(self, msg: Image) -> None:
        self.image = msg

    def on_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    # -- board plane from the camera --------------------------------------

    def board_plane(self):
        """Return (normal, offset) of the board plane in the camera frame."""
        frame = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            grey, (self.cols, self.rows),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return None, "checkerboard not found in the image"

        corners = cv2.cornerSubPix(
            grey, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        K = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        D = np.array(self.camera_info.d, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points, corners, K, D,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, "solvePnP failed"

        R, _ = cv2.Rodrigues(rvec)
        # Board z axis is the plane normal; the plane passes through the board
        # origin at tvec.
        normal = R[:, 2]
        normal = normal / np.linalg.norm(normal)
        offset = float(normal @ tvec.ravel())
        return (normal, offset), None

    # -- board points from the LiDAR --------------------------------------

    def board_points(self):
        """RANSAC the board line out of the scan; return points in the LiDAR frame."""
        max_range = float(self.get_parameter("scan_max_range").value)
        half_fov = float(self.get_parameter("scan_half_fov").value)
        threshold = float(self.get_parameter("ransac_threshold").value)
        min_points = int(self.get_parameter("min_board_points").value)

        pts = []
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            a = angle
            angle += self.scan.angle_increment
            if not math.isfinite(r) or r <= self.scan.range_min or r > max_range:
                continue
            if abs(math.atan2(math.sin(a), math.cos(a))) > half_fov:
                continue
            pts.append((r * math.cos(a), r * math.sin(a)))

        if len(pts) < min_points:
            return None, f"only {len(pts)} scan points in the capture window"

        data = np.array(pts)
        rng = np.random.default_rng(0)
        best_inliers = None
        for _ in range(200):
            i, j = rng.choice(len(data), size=2, replace=False)
            p, q = data[i], data[j]
            d = q - p
            norm = np.linalg.norm(d)
            if norm < 1e-6:
                continue
            n = np.array([-d[1], d[0]]) / norm
            dist = np.abs((data - p) @ n)
            inliers = dist < threshold
            if best_inliers is None or inliers.sum() > best_inliers.sum():
                best_inliers = inliers

        if best_inliers is None or best_inliers.sum() < min_points:
            return None, "no consistent line found in the scan"

        board = data[best_inliers]
        # 2D LiDAR: all returns lie in the sensor's z = 0 plane.
        return np.hstack([board, np.zeros((len(board), 1))]), None

    # -- services ----------------------------------------------------------

    def on_capture(self, request, response):
        del request
        if self.image is None or self.camera_info is None or self.scan is None:
            response.success = False
            response.message = "waiting for image, camera_info and scan"
            return response

        plane, err = self.board_plane()
        if plane is None:
            response.success = False
            response.message = err
            return response

        points, err = self.board_points()
        if points is None:
            response.success = False
            response.message = err
            return response

        normal, offset = plane
        self.observations.append({
            "plane_normal": normal.tolist(),
            "plane_offset": offset,
            "lidar_points": points.tolist(),
        })
        response.success = True
        response.message = (
            f"captured pose {len(self.observations)}: {len(points)} LiDAR "
            f"points, board at {offset:.2f} m")
        self.get_logger().info(response.message)
        return response

    def on_save(self, request, response):
        del request
        if len(self.observations) < 3:
            response.success = False
            response.message = (
                f"only {len(self.observations)} poses captured; the solver "
                f"needs at least 3 with differing orientations")
            return response

        os.makedirs(os.path.dirname(self.output) or ".", exist_ok=True)
        with open(self.output, "w", encoding="utf-8") as fh:
            json.dump({
                "description": "Captured from live camera and LiDAR topics.",
                "observations": self.observations,
            }, fh)

        response.success = True
        response.message = (
            f"wrote {self.output} with {len(self.observations)} board poses")
        self.get_logger().info(response.message)
        return response


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = CalibCollector()
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
