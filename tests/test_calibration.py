#!/usr/bin/env python3
"""Checks on the camera-to-LiDAR extrinsic calibration.

The residual on its own proves very little: a solver that converges to the
wrong transform can still report a small residual, and one that overfits a
degenerate set of board poses reports the smallest residual of all. These tests
check the things that actually matter.

    python3 tests/test_calibration.py
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "src", "shuttle_perception"),
)

from shuttle_perception.calibrate_extrinsics import (  # noqa: E402
    calibrate, rotation_from_rotvec, rotvec_from_rotation,
)

FAILURES = []


def check(condition, message):
    if condition:
        print(f"ok   {message}")
    else:
        print(f"FAIL {message}")
        FAILURES.append(message)


def generate(tmpdir, **kwargs):
    out = os.path.join(tmpdir, "obs.json")
    cmd = [sys.executable, "-m", "shuttle_perception.synth_calib_data",
           "--out", out]
    for key, value in kwargs.items():
        cmd += [f"--{key.replace('_', '-')}", str(value)]
    env = dict(os.environ)
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    env["PYTHONPATH"] = os.path.join(root, "src", "shuttle_perception")
    subprocess.run(cmd, check=True, capture_output=True, env=env)
    import json
    with open(out, encoding="utf-8") as fh:
        return json.load(fh)


def test_rotation_roundtrip():
    rng = np.random.default_rng(7)
    worst = 0.0
    for _ in range(200):
        v = rng.normal(0.0, 1.0, size=3)
        v = v / np.linalg.norm(v) * rng.uniform(0.0, math.pi * 0.99)
        back = rotvec_from_rotation(rotation_from_rotvec(v))
        worst = max(worst, float(np.linalg.norm(back - v)))
    check(worst < 1e-8,
          f"axis-angle round trip is exact (worst {worst:.2e})")


def test_recovers_known_transform():
    with tempfile.TemporaryDirectory() as tmp:
        data = generate(tmp)
        report = calibrate(data["observations"], verbose=False)

    truth = data["ground_truth"]
    t_true = np.array([truth["translation"]["x"], truth["translation"]["y"],
                       truth["translation"]["z"]])
    t_est = np.array([report["translation_m"]["x"],
                      report["translation_m"]["y"],
                      report["translation_m"]["z"]])
    R_true = rotation_from_rotvec(np.array(truth["rotvec"]))
    R_est = np.array(report["rotation_matrix"])
    angle = float(np.linalg.norm(rotvec_from_rotation(R_true.T @ R_est)))
    trans_err = float(np.linalg.norm(t_est - t_true))

    check(report["converged"], "solver converged")
    check(trans_err < 0.02,
          f"translation recovered to better than 20 mm ({trans_err * 1000:.1f} mm)")
    check(math.degrees(angle) < 1.0,
          f"rotation recovered to better than 1 deg ({math.degrees(angle):.3f} deg)")

    # The residual should sit at the noise floor. Much larger means the model is
    # wrong; much smaller means it is fitting noise.
    sigma = data["noise"]["sigma_lidar_m"]
    rms = report["residual_rms_m"]
    check(0.5 * sigma < rms < 2.0 * sigma,
          f"residual is at the LiDAR noise floor "
          f"(RMS {rms * 1000:.1f} mm vs sigma {sigma * 1000:.1f} mm)")
    check(report["jacobian_condition_number"] < 1e4,
          f"pose set is well conditioned "
          f"({report['jacobian_condition_number']:.0f})")


def test_more_noise_gives_bigger_residual():
    with tempfile.TemporaryDirectory() as tmp:
        quiet = calibrate(generate(tmp, sigma_lidar=0.004)["observations"],
                          verbose=False)
    with tempfile.TemporaryDirectory() as tmp:
        loud = calibrate(generate(tmp, sigma_lidar=0.030)["observations"],
                         verbose=False)
    check(loud["residual_rms_m"] > quiet["residual_rms_m"] * 2.0,
          f"residual tracks the noise level "
          f"({quiet['residual_rms_m'] * 1000:.1f} mm -> "
          f"{loud['residual_rms_m'] * 1000:.1f} mm)")


def test_rejects_too_few_poses():
    with tempfile.TemporaryDirectory() as tmp:
        data = generate(tmp, poses=6)
    try:
        calibrate(data["observations"][:2], verbose=False)
        check(False, "two board poses are rejected")
    except ValueError:
        check(True, "two board poses are rejected")


def test_detects_degenerate_poses():
    """Near-parallel boards leave rotation about the normal unobservable."""
    rng = np.random.default_rng(3)
    normal = np.array([0.0, 0.0, -1.0])
    observations = []
    for i in range(8):
        # All boards nearly fronto-parallel: normals barely differ.
        n = normal + rng.normal(0.0, 0.002, size=3)
        n /= np.linalg.norm(n)
        points = rng.normal(0.0, 0.4, size=(60, 3))
        points[:, 2] = 2.0 + 0.01 * i
        observations.append({
            "plane_normal": n.tolist(),
            "plane_offset": float(-2.0 - 0.01 * i),
            "lidar_points": points.tolist(),
        })
    report = calibrate(observations, verbose=False)
    check(report["jacobian_condition_number"] > 1e4,
          f"degenerate pose set is flagged as ill-conditioned "
          f"({report['jacobian_condition_number']:.3g})")


def main() -> int:
    print("camera-to-LiDAR calibration tests\n")
    test_rotation_roundtrip()
    test_recovers_known_transform()
    test_more_noise_gives_bigger_residual()
    test_rejects_too_few_poses()
    test_detects_degenerate_poses()
    print(f"\n{len(FAILURES)} failures")
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
