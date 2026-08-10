#!/usr/bin/env python3
"""Generate synthetic camera-to-LiDAR calibration observations.

Produces the JSON that calibrate_extrinsics.py consumes, from a known
extrinsic transform, so the calibration can be checked against an answer rather
than only against its own residual. A solver that converges neatly to the wrong
transform still reports a small residual; comparing against ground truth is the
only way to catch that.

Noise model:
  * LiDAR range noise pushes each board point off the true plane along the beam
    direction (sigma_lidar, metres);
  * camera plane estimates carry angular error from corner localisation and PnP
    (sigma_normal, radians) and a small offset error (sigma_offset, metres).

Schema:
{
  "ground_truth": {"translation": {...}, "rotvec": [rx, ry, rz]},
  "observations": [
     {"plane_normal": [nx, ny, nz],   # unit normal in the CAMERA frame
      "plane_offset": d,              # metres, n . p = d for p on the board
      "lidar_points": [[x, y, z], ...]}  # board points in the LIDAR frame
  ]
}
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

from shuttle_perception.calibrate_extrinsics import rotation_from_rotvec


def make_board_points(rng, centre, normal, half_size, count):
    """Sample points on a planar board patch in the camera frame."""
    normal = normal / np.linalg.norm(normal)
    # Any two vectors spanning the plane.
    helper = np.array([0.0, 0.0, 1.0])
    if abs(float(normal @ helper)) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, helper)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)

    a = rng.uniform(-half_size, half_size, size=count)
    b = rng.uniform(-half_size, half_size, size=count)
    return centre[None, :] + a[:, None] * u[None, :] + b[:, None] * v[None, :]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="calibration/observations.json")
    parser.add_argument("--poses", type=int, default=12,
                        help="number of board orientations")
    parser.add_argument("--points-per-pose", type=int, default=120)
    parser.add_argument("--sigma-lidar", type=float, default=0.012,
                        help="LiDAR range noise, m")
    parser.add_argument("--sigma-normal", type=float, default=0.004,
                        help="camera plane normal error, rad")
    parser.add_argument("--sigma-offset", type=float, default=0.003,
                        help="camera plane offset error, m")
    parser.add_argument("--seed", type=int, default=20260810)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    # True extrinsic: the transform from lidar_link to camera_optical_frame
    # implied by the URDF, plus a small mounting error of the kind calibration
    # exists to find.
    #
    # In the URDF the LiDAR sits at (0.24, 0, 0.27) and the camera body at
    # (0.40, 0, 0.24) in base_link, and the optical frame is rotated by
    # rpy(-pi/2, 0, -pi/2). Composing those gives the nominal transform below.
    nominal_rotvec = np.array([-1.2091996, 1.2091996, -1.2091996])  # ~120 deg
    mounting_error = np.array([0.008, -0.011, 0.006])  # rad, a few degrees
    true_rotvec = nominal_rotvec + mounting_error
    true_translation = np.array([0.021, -0.034, 0.161])

    R = rotation_from_rotvec(true_rotvec)
    t = true_translation

    observations = []
    for i in range(args.poses):
        # Spread the board orientations. Rotation is only observable if the
        # normals are not all parallel, so sweep both tilt axes rather than
        # nodding the board about one.
        yaw = rng.uniform(-0.5, 0.5)
        pitch = rng.uniform(-0.45, 0.45)
        normal_cam = np.array([
            math.sin(yaw) * math.cos(pitch),
            math.sin(pitch),
            -math.cos(yaw) * math.cos(pitch),
        ])
        normal_cam /= np.linalg.norm(normal_cam)

        distance = rng.uniform(1.4, 3.2)
        centre_cam = -normal_cam * distance
        centre_cam = centre_cam + rng.normal(0.0, 0.12, size=3)

        points_cam = make_board_points(
            rng, centre_cam, normal_cam, half_size=0.35,
            count=args.points_per_pose)

        # True plane in the camera frame.
        offset_true = float(normal_cam @ centre_cam)

        # LiDAR sees those same physical points, expressed in the LiDAR frame.
        points_lidar = (points_cam - t) @ R
        # Range noise acts along the beam, i.e. radially from the LiDAR origin.
        ranges = np.linalg.norm(points_lidar, axis=1, keepdims=True)
        directions = points_lidar / np.maximum(ranges, 1e-9)
        points_lidar = points_lidar + directions * rng.normal(
            0.0, args.sigma_lidar, size=(len(points_lidar), 1))

        # Camera plane estimate: perturb the normal by a small rotation and the
        # offset by a small scalar error.
        perturb = rng.normal(0.0, args.sigma_normal, size=3)
        normal_obs = rotation_from_rotvec(perturb) @ normal_cam
        normal_obs /= np.linalg.norm(normal_obs)
        offset_obs = offset_true + rng.normal(0.0, args.sigma_offset)

        observations.append({
            "plane_normal": normal_obs.tolist(),
            "plane_offset": float(offset_obs),
            "lidar_points": points_lidar.tolist(),
        })

    payload = {
        "description": (
            "Synthetic camera-to-LiDAR calibration observations. "
            "Generated by synth_calib_data.py; see calibration/procedure.md "
            "for how the equivalent data is captured on a real robot."
        ),
        "seed": args.seed,
        "noise": {
            "sigma_lidar_m": args.sigma_lidar,
            "sigma_normal_rad": args.sigma_normal,
            "sigma_offset_m": args.sigma_offset,
        },
        "ground_truth": {
            "translation": {
                "x": float(t[0]), "y": float(t[1]), "z": float(t[2]),
            },
            "rotvec": true_rotvec.tolist(),
        },
        "observations": observations,
    }

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)

    print(f"Wrote {args.out}: {args.poses} board poses, "
          f"{args.poses * args.points_per_pose} LiDAR points, seed {args.seed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
