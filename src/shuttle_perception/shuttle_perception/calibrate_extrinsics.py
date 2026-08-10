#!/usr/bin/env python3
"""Camera-to-LiDAR extrinsic calibration from a planar target.

Method
------
A checkerboard is held in front of the robot in several poses. For each pose:

  * the camera sees the board corners, and with known intrinsics a PnP solve
    gives the board plane in the camera frame as a normal `n` and offset `d`,
    so every point `p` on the board satisfies `n . p = d`;
  * the LiDAR returns a set of points that lie on that same physical board.

The unknown is the rigid transform `T_cam_lidar` that maps LiDAR points into
the camera frame. The correct transform puts every LiDAR board point on the
camera's board plane, so the residual for LiDAR point `q` in observation `i` is

    r = n_i . (R q + t) - d_i

which is a point-to-plane error in metres. Minimising the sum of squares over
all points and observations is a 6-parameter non-linear least squares problem,
solved here with Levenberg-Marquardt over an axis-angle rotation.

Why point-to-plane and not point-to-point: nothing associates a particular
LiDAR return with a particular pixel. The plane constraint is the strongest
correspondence-free constraint available, and it is what makes the problem
solvable from a target that is just a flat board.

Observability
-------------
A single board pose constrains only the direction normal to it. At least three
non-parallel board orientations are needed to fix all three rotational degrees
of freedom, and translation along a direction lying in every observed plane is
never observable. The script reports the Jacobian's condition number and warns
when the set of poses is degenerate, rather than silently returning a confident
answer to an under-determined problem.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares


# --------------------------------------------------------------------------
# Rotation helpers (axis-angle <-> matrix), Rodrigues' formula
# --------------------------------------------------------------------------

def rotation_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3)
    axis = rotvec / theta
    kx, ky, kz = axis
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def rotvec_from_rotation(R: np.ndarray) -> np.ndarray:
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return np.zeros(3)
    if abs(theta - math.pi) < 1e-6:
        # Near 180 degrees the antisymmetric part vanishes; recover the axis
        # from the symmetric part instead.
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        if R[2, 1] - R[1, 2] < 0:
            axis[0] = -axis[0]
        return axis / np.linalg.norm(axis) * theta
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w / (2.0 * math.sin(theta)) * theta


def euler_from_rotation(R: np.ndarray) -> tuple[float, float, float]:
    """Roll, pitch, yaw (XYZ fixed-axis), for human-readable reporting."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


# --------------------------------------------------------------------------
# Solver
# --------------------------------------------------------------------------

def residuals(params: np.ndarray, observations) -> np.ndarray:
    """Point-to-plane residual, in metres, for every LiDAR board point."""
    R = rotation_from_rotvec(params[:3])
    t = params[3:6]
    out = []
    for obs in observations:
        normal = np.asarray(obs["plane_normal"], dtype=float)
        offset = float(obs["plane_offset"])
        points = np.asarray(obs["lidar_points"], dtype=float)
        transformed = points @ R.T + t
        out.append(transformed @ normal - offset)
    return np.concatenate(out) if out else np.zeros(0)


def calibrate(observations, initial_guess=None, verbose=True):
    if len(observations) < 3:
        raise ValueError(
            f"need at least 3 board poses to constrain rotation, got "
            f"{len(observations)}")

    x0 = np.zeros(6)
    if initial_guess is not None:
        x0 = np.asarray(initial_guess, dtype=float)

    result = least_squares(
        residuals, x0, args=(observations,), method="lm", xtol=1e-12,
        ftol=1e-12, max_nfev=20000)

    R = rotation_from_rotvec(result.x[:3])
    t = result.x[3:6]
    r = result.fun
    n = len(r)

    rms = float(np.sqrt(np.mean(r ** 2)))
    mean_abs = float(np.mean(np.abs(r)))
    max_abs = float(np.max(np.abs(r)))

    # Observability: the Gauss-Newton normal matrix J^T J is singular when a
    # degree of freedom is unconstrained by the chosen board poses.
    J = result.jac
    JTJ = J.T @ J
    singular = np.linalg.svd(JTJ, compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float("inf")

    # Parameter covariance, scaled by the residual variance. Degrees of freedom
    # is (residuals - parameters).
    std = None
    dof = n - 6
    if dof > 0 and np.isfinite(condition) and condition < 1e12:
        sigma2 = float(r @ r) / dof
        cov = sigma2 * np.linalg.inv(JTJ)
        std = np.sqrt(np.diag(cov))

    roll, pitch, yaw = euler_from_rotation(R)

    report = {
        "translation_m": {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
        "rotation_rpy_rad": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "rotation_rpy_deg": {
            "roll": math.degrees(roll),
            "pitch": math.degrees(pitch),
            "yaw": math.degrees(yaw),
        },
        "rotation_matrix": R.tolist(),
        "residual_rms_m": rms,
        "residual_mean_abs_m": mean_abs,
        "residual_max_abs_m": max_abs,
        "num_observations": len(observations),
        "num_points": int(n),
        "jacobian_condition_number": condition,
        "converged": bool(result.success),
    }
    if std is not None:
        report["parameter_std"] = {
            "rotvec_x_rad": float(std[0]),
            "rotvec_y_rad": float(std[1]),
            "rotvec_z_rad": float(std[2]),
            "translation_x_m": float(std[3]),
            "translation_y_m": float(std[4]),
            "translation_z_m": float(std[5]),
        }

    if verbose:
        print(f"Solved from {len(observations)} board poses, {n} LiDAR points")
        print(f"  translation : x {t[0]:+.4f}  y {t[1]:+.4f}  z {t[2]:+.4f}  m")
        print(f"  rotation    : roll {math.degrees(roll):+.3f}  "
              f"pitch {math.degrees(pitch):+.3f}  "
              f"yaw {math.degrees(yaw):+.3f}  deg")
        print(f"  residual    : RMS {rms * 1000:.2f} mm, "
              f"mean |r| {mean_abs * 1000:.2f} mm, max {max_abs * 1000:.2f} mm")
        print(f"  condition   : {condition:.1f}")
        if condition > 1e4:
            print("  WARNING: the board poses are close to degenerate. Add "
                  "orientations that are not near-parallel to the existing "
                  "ones; some degrees of freedom are barely observable.")

    return report


def write_yaml(report, path, lidar_frame="lidar_link",
               camera_frame="camera_optical_frame"):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    t = report["translation_m"]
    r = report["rotation_rpy_rad"]
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(
            "# Camera-to-LiDAR extrinsic calibration.\n"
            "# Generated by shuttle_perception/calibrate_extrinsics.py.\n"
            "#\n"
            f"# Transform maps points from {lidar_frame} into {camera_frame}.\n"
            f"# Point-to-plane residual: RMS "
            f"{report['residual_rms_m'] * 1000:.2f} mm over "
            f"{report['num_points']} points from "
            f"{report['num_observations']} board poses.\n"
            "#\n"
            "# Publish with:\n"
            "#   ros2 run tf2_ros static_transform_publisher \\\n"
            f"#     {t['x']:.6f} {t['y']:.6f} {t['z']:.6f} "
            f"{r['yaw']:.6f} {r['pitch']:.6f} {r['roll']:.6f} \\\n"
            f"#     {lidar_frame} {camera_frame}\n"
            "camera_lidar_extrinsics:\n"
            f"  parent_frame: {lidar_frame}\n"
            f"  child_frame: {camera_frame}\n"
            "  translation:\n"
            f"    x: {t['x']:.6f}\n"
            f"    y: {t['y']:.6f}\n"
            f"    z: {t['z']:.6f}\n"
            "  rotation_rpy:\n"
            f"    roll: {r['roll']:.6f}\n"
            f"    pitch: {r['pitch']:.6f}\n"
            f"    yaw: {r['yaw']:.6f}\n"
            "  residual:\n"
            f"    rms_m: {report['residual_rms_m']:.6f}\n"
            f"    mean_abs_m: {report['residual_mean_abs_m']:.6f}\n"
            f"    max_abs_m: {report['residual_max_abs_m']:.6f}\n"
            f"    num_points: {report['num_points']}\n"
            f"    num_observations: {report['num_observations']}\n"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observations", required=True,
                        help="JSON file of board observations "
                             "(see synth_calib_data.py for the schema)")
    parser.add_argument("--out-yaml", default="calibration/extrinsics.yaml")
    parser.add_argument("--out-json", default="")
    args = parser.parse_args()

    if not os.path.exists(args.observations):
        print(f"error: {args.observations} not found", file=sys.stderr)
        return 1

    with open(args.observations, encoding="utf-8") as fh:
        data = json.load(fh)
    observations = data["observations"]

    report = calibrate(observations)

    # If the file records the transform used to generate it, report the error
    # against it. This is what makes the residual meaningful rather than merely
    # small: a solver can drive the residual to zero at the wrong answer.
    truth = data.get("ground_truth")
    if truth:
        t_true = np.array([truth["translation"]["x"], truth["translation"]["y"],
                           truth["translation"]["z"]])
        t_est = np.array([report["translation_m"]["x"],
                          report["translation_m"]["y"],
                          report["translation_m"]["z"]])
        R_true = rotation_from_rotvec(np.array(truth["rotvec"]))
        R_est = np.array(report["rotation_matrix"])
        angle = float(np.linalg.norm(rotvec_from_rotation(R_true.T @ R_est)))
        trans_err = float(np.linalg.norm(t_est - t_true))
        report["error_vs_ground_truth"] = {
            "translation_m": trans_err,
            "rotation_rad": angle,
            "rotation_deg": math.degrees(angle),
        }
        print(f"  vs truth    : translation {trans_err * 1000:.2f} mm, "
              f"rotation {math.degrees(angle):.4f} deg")

    write_yaml(report, args.out_yaml)
    print(f"Wrote {args.out_yaml}")

    if args.out_json:
        os.makedirs(os.path.dirname(args.out_json) or ".", exist_ok=True)
        with open(args.out_json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2)
        print(f"Wrote {args.out_json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
