#!/usr/bin/env bash
#
# Reproduce the headline controller comparison end to end.
#
#   1. run the controller core unit tests
#   2. tune both controllers by grid search on the tuning scenarios
#   3. run every controller over every scenario, speed and noise setting
#   4. generate the figures and docs/RESULTS.md from the raw CSV
#
# Everything written under results/ is derived; delete it and re-run to rebuild.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

OUT_DIR="${OUT_DIR:-results/bench}"

# Always build from the source actually present. The Docker image prebuilds the
# core, but the repository is bind-mounted over it at run time, so the prebuilt
# binary can be older than the source. Publishing numbers produced by stale code
# would defeat the point of the whole exercise.
BUILD_DIR="${BUILD_DIR:-/tmp/shuttle-core-build}"

echo "Building the controller core from source..."
cmake -S src/shuttle_control/core -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DSHUTTLE_CORE_BUILD_TESTS=ON >/dev/null
cmake --build "$BUILD_DIR" -j"$(nproc 2>/dev/null || echo 4)" >/dev/null

BENCH_BIN="$BUILD_DIR/controller_bench"
TEST_BIN="$BUILD_DIR/test_core"

mkdir -p "$OUT_DIR/timeseries"

echo "== Unit tests =="
"$TEST_BIN"

echo
echo "== Tuning and comparison =="
"$BENCH_BIN" --out "$OUT_DIR" --tune

# Keep the tuned gains next to the ROS 2 parameters so simulation and bench use
# the same numbers.
cp "$OUT_DIR/tuned_gains.yaml" src/shuttle_bringup/config/tuned_gains.yaml
echo "Copied tuned gains to src/shuttle_bringup/config/tuned_gains.yaml"

echo
echo "== Analysis =="
python3 analysis/compare_controllers.py \
  --metrics "$OUT_DIR/metrics.csv" \
  --timeseries "$OUT_DIR/timeseries" \
  --figures analysis/figures \
  --results docs/RESULTS.md

echo
echo "Done. See docs/RESULTS.md and analysis/figures/."
