#!/usr/bin/env bash
#
# Everything that can be checked without ROS or Gazebo. This is what CI runs.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BUILD_DIR="${BUILD_DIR:-build/core}"
BENCH_BIN="/build/controller_bench"
TEST_BIN="/build/test_core"

if [ ! -x "$TEST_BIN" ]; then
  echo "== Building the controller core =="
  cmake -S src/shuttle_control/core -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release -DSHUTTLE_CORE_BUILD_TESTS=ON >/dev/null
  cmake --build "$BUILD_DIR" -j"$(nproc 2>/dev/null || echo 4)" >/dev/null
  BENCH_BIN="$REPO_ROOT/$BUILD_DIR/controller_bench"
  TEST_BIN="$REPO_ROOT/$BUILD_DIR/test_core"
fi

failed=0

echo "== Controller core tests =="
"$TEST_BIN" || failed=1

echo
echo "== Scenario geometry: C++ against Python =="
paths_dir="$(mktemp -d)"
"$BENCH_BIN" --dump-paths "$paths_dir" >/dev/null
python3 tests/test_scenarios_match.py --paths "$paths_dir" || failed=1
rm -rf "$paths_dir"

echo
echo "== ROS 2 parameter files =="
python3 tests/test_param_files.py || failed=1

echo
echo "== Calibration tests =="
PYTHONPATH="$REPO_ROOT/src/shuttle_perception:${PYTHONPATH:-}" \
  python3 tests/test_calibration.py || failed=1

echo
if [ "$failed" -ne 0 ]; then
  echo "FAILED"
  exit 1
fi
echo "All checks passed."
