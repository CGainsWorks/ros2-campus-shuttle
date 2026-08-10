#!/usr/bin/env bash
#
# Reproduce the camera-to-LiDAR extrinsic calibration and its reported residual.
# Needs no hardware and no ROS: the observations are generated from a known
# transform so the solve can be scored against a true answer.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

export PYTHONPATH="$REPO_ROOT/src/shuttle_perception:${PYTHONPATH:-}"

mkdir -p calibration

echo "== Generating observations =="
python3 -m shuttle_perception.synth_calib_data \
  --out calibration/observations.json

echo
echo "== Solving =="
python3 -m shuttle_perception.calibrate_extrinsics \
  --observations calibration/observations.json \
  --out-yaml calibration/extrinsics.yaml \
  --out-json calibration/extrinsics_report.json

echo
echo "== Tests =="
python3 tests/test_calibration.py
