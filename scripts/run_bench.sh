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

BENCH_BIN="${BENCH_BIN:-controller_bench}"
TEST_BIN="${TEST_BIN:-test_core}"
OUT_DIR="${OUT_DIR:-results/bench}"

if ! command -v "$BENCH_BIN" >/dev/null 2>&1; then
  echo "Building the controller core..."
  cmake -S src/shuttle_control/core -B build/core \
    -DCMAKE_BUILD_TYPE=Release -DSHUTTLE_CORE_BUILD_TESTS=ON >/dev/null
  cmake --build build/core -j"$(nproc 2>/dev/null || echo 4)" >/dev/null
  BENCH_BIN="$REPO_ROOT/build/core/controller_bench"
  TEST_BIN="$REPO_ROOT/build/core/test_core"
fi

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
