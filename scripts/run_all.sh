#!/usr/bin/env bash
#
# Everything that does not need ROS, end to end, with the outputs verified.
#
#   1. controller core unit tests
#   2. scenario geometry cross-check (C++ against Python)
#   3. ROS 2 parameter file validation
#   4. camera-to-LiDAR calibration and its tests
#   5. controller tuning, the full comparison matrix, figures and RESULTS.md
#   6. a check that every expected artifact exists and is not empty
#
# The final check matters. Each step above prints its own success but a script
# that runs to completion is not the same as a script that produced the results
# it claims to: a plotting call that silently wrote nothing, or a stale file
# left from a previous run, both look like success from the console output.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

failed=0
step() { echo; echo "================ $1 ================"; }

step "Tests"
bash scripts/run_tests.sh || failed=1

step "Camera-to-LiDAR calibration"
bash scripts/run_calibration.sh || failed=1

step "Controller comparison"
bash scripts/run_bench.sh || failed=1

step "Verifying artifacts"
check_file() {
  local path="$1"
  local min_bytes="${2:-1}"
  if [ ! -f "$path" ]; then
    echo "  MISSING  $path"
    failed=1
    return
  fi
  local size
  size=$(wc -c < "$path")
  if [ "$size" -lt "$min_bytes" ]; then
    echo "  EMPTY    $path ($size bytes, expected at least $min_bytes)"
    failed=1
    return
  fi
  echo "  ok       $path ($size bytes)"
}

check_file results/bench/metrics.csv 1000
check_file results/bench/tuned_gains.yaml 200
check_file src/shuttle_bringup/config/tuned_gains.yaml 200
check_file docs/RESULTS.md 2000
check_file analysis/figures/trajectories.png 10000
check_file analysis/figures/step_response.png 10000
check_file analysis/figures/rms_by_scenario.png 10000
check_file analysis/figures/speed_and_effort.png 10000
check_file calibration/extrinsics.yaml 400
check_file calibration/extrinsics_report.json 400

# Every benchmark run must have finished cleanly, or the comparison is not a
# comparison.
echo
python3 - <<'PY' || failed=1
import csv
import sys

bad = []
total = 0
with open("results/bench/metrics.csv") as fh:
    for row in csv.DictReader(fh):
        total += 1
        if (row["goal_reached"] != "1" or row["diverged"] == "1"
                or row["stalled"] == "1"):
            bad.append(f"{row['controller']} {row['scenario']} "
                       f"@ {row['target_speed']} noise={row['noise']}")

if total == 0:
    print("  no benchmark runs recorded")
    sys.exit(1)
if bad:
    print(f"  {len(bad)} of {total} runs did not finish cleanly:")
    for entry in bad:
        print(f"    {entry}")
    sys.exit(1)
print(f"  ok       all {total} benchmark runs finished cleanly")
PY

# The number of timeseries files should match the number of runs.
runs=$(($(wc -l < results/bench/metrics.csv) - 1))
series=$(find results/bench/timeseries -name '*.csv' 2>/dev/null | wc -l)
if [ "$runs" -ne "$series" ]; then
  echo "  MISMATCH $runs metric rows but $series time series files"
  failed=1
else
  echo "  ok       $runs runs, $series time series"
fi

echo
if [ "$failed" -ne 0 ]; then
  echo "END-TO-END FAILED"
  exit 1
fi
echo "END-TO-END PASSED"
echo
echo "Headline result: docs/RESULTS.md"
echo "Figures:         analysis/figures/"
echo "Calibration:     calibration/extrinsics.yaml"
