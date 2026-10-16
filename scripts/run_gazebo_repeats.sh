#!/usr/bin/env bash
#
# The Gazebo cross-check, repeated and summarised into the committed results.
#
# Gazebo runs are not bit-reproducible: sensor noise, transport timing and the
# physics step all vary between runs. A single run "showing" a winner can
# therefore be noise, so the cross-check is run several times and every repeat
# is reported, not just one.
#
#   scripts/run_gazebo_repeats.sh [repeats] [scenario] [speed]
#
# Defaults: 3 repeats of `corner` at 1.0 m/s, the configuration the README
# quotes. Writes
#   results/gazebo/repeat_N/       raw output of each repeat (not committed)
#   results/gazebo/metrics.csv     every repeat's rows together (not committed)
#   docs/RESULTS_GAZEBO.md and analysis/figures/gazebo/   from all repeats
#   results/controller_comparison.json   the gazebo section (committed)
#
# Must run inside the simulation image:
#   docker compose --profile sim run --rm sim scripts/run_gazebo_repeats.sh
set -euo pipefail

REPEATS="${1:-3}"
SCENARIO="${2:-corner}"
SPEED="${3:-1.0}"
BASE="results/gazebo"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

rm -rf "$BASE"
mkdir -p "$BASE/timeseries"

for i in $(seq 1 "$REPEATS"); do
  echo
  echo "################ repeat $i of $REPEATS ################"
  OUT_DIR="$BASE/repeat_$i" SKIP_ANALYSIS=1 \
    bash scripts/run_comparison.sh "$SCENARIO" "$SPEED"
done

# All repeats in one metrics file: the per-scenario table in RESULTS_GAZEBO.md
# then shows the mean over repeats. The figures use the last repeat's series.
head -n 1 "$BASE/repeat_1/metrics.csv" > "$BASE/metrics.csv"
for i in $(seq 1 "$REPEATS"); do
  tail -n +2 "$BASE/repeat_$i/metrics.csv" >> "$BASE/metrics.csv"
done
cp "$BASE/repeat_$REPEATS"/timeseries/*.csv "$BASE/timeseries/"

# Record what produced the numbers. gz reports its own version.
python3 - "$BASE/run_info.json" <<'PY'
import datetime
import json
import os
import subprocess
import sys

try:
    gz = subprocess.run(["gz", "sim", "--versions"], capture_output=True,
                        text=True, check=True).stdout.strip()
except (OSError, subprocess.CalledProcessError):
    gz = "unknown"
info = {
    "simulator": f"Gazebo Harmonic (gz-sim {gz}) with ROS 2 {os.environ.get('ROS_DISTRO', '?').capitalize()}",
    "date": datetime.date.today().isoformat(),
}
with open(sys.argv[1], "w", encoding="utf-8", newline="\n") as fh:
    json.dump(info, fh, indent=2)
    fh.write("\n")
PY

echo
echo "== Analysis =="
python3 analysis/compare_controllers.py \
  --metrics "$BASE/metrics.csv" \
  --timeseries "$BASE/timeseries" \
  --figures analysis/figures/gazebo \
  --results docs/RESULTS_GAZEBO.md \
  --reference-speed "$SPEED"

python3 analysis/summarise_results.py --gazebo "$BASE"

echo
echo "Done. See docs/RESULTS_GAZEBO.md and results/controller_comparison.json"
