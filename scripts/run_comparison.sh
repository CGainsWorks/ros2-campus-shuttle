#!/usr/bin/env bash
#
# Gazebo cross-check of the controller comparison.
#
# Runs each controller over the same scenario in the physics simulator, scoring
# against the Gazebo ground-truth pose. This is the high-fidelity counterpart to
# scripts/run_bench.sh: slower, heavier, and it exercises wheel slip and contact
# dynamics the kinematic benchmark does not model.
#
# The two controllers get the same world, the same latched plan and the same
# start pose. Only the lateral control law differs.
#
# Usage:
#   scripts/run_comparison.sh [scenario] [speed]
#
# Must run inside the simulation image (docker/Dockerfile), which has ROS 2 and
# Gazebo. From the host:
#   docker compose run --rm sim scripts/run_comparison.sh campus_loop 1.0
set -euo pipefail

SCENARIO="${1:-campus_loop}"
SPEED="${2:-1.0}"
OUT_DIR="${OUT_DIR:-results/gazebo}"
TIMEOUT="${TIMEOUT:-400}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: ros2 not found. Run this inside the simulation image:" >&2
  echo "  docker compose run --rm sim scripts/run_comparison.sh $SCENARIO $SPEED" >&2
  exit 1
fi

# Start poses, matching the scenario definitions in scenario_paths.py. A
# controller started somewhere other than its scenario's start pose is not
# running the experiment that was designed.
case "$SCENARIO" in
  straight_offset) X=0.0;  Y=1.0; YAW=0.0 ;;
  heading_offset)  X=0.0;  Y=0.0; YAW=0.5235988 ;;
  circle)          X=6.0;  Y=0.0; YAW=1.5707963 ;;
  corner)          X=0.0;  Y=0.0; YAW=0.0 ;;
  figure_eight)    X=0.0;  Y=0.0; YAW=0.7853982 ;;
  campus_loop)     X=3.0;  Y=0.0; YAW=0.0 ;;
  sharp_turns)     X=0.0;  Y=0.0; YAW=0.0 ;;
  *)
    echo "error: unknown scenario '$SCENARIO'" >&2
    echo "expected one of: straight_offset heading_offset circle corner \
figure_eight campus_loop sharp_turns" >&2
    exit 2
    ;;
esac

mkdir -p "$OUT_DIR/timeseries"
# Start from a clean metrics file so a rerun does not append to stale rows.
rm -f "$OUT_DIR/metrics.csv"

for CONTROLLER in pure_pursuit stanley; do
  echo
  echo "=============================================================="
  echo " $CONTROLLER on $SCENARIO at $SPEED m/s"
  echo "=============================================================="

  # The launch file exits once the metrics recorder has written its output.
  # The timeout is a backstop for a run that never reaches its goal.
  set +e
  timeout --signal=INT "$TIMEOUT" \
    ros2 launch shuttle_bringup comparison.launch.py \
      controller:="$CONTROLLER" \
      scenario:="$SCENARIO" \
      speed:="$SPEED" \
      output_dir:="$OUT_DIR" \
      x:="$X" y:="$Y" yaw:="$YAW"
  status=$?
  set -e

  if [ $status -eq 124 ]; then
    echo "warning: $CONTROLLER hit the ${TIMEOUT}s timeout on $SCENARIO" >&2
  fi

  # Gazebo can leave processes behind when a launch is interrupted; a stale
  # gzserver holds the port and the next run silently attaches to the old world.
  pkill -f gzserver >/dev/null 2>&1 || true
  pkill -f gzclient >/dev/null 2>&1 || true
  sleep 3

  # Every controller must have contributed a row. If the robot failed to spawn,
  # or ground truth never arrived, the recorder writes nothing -- and without
  # this check the script would go on to "compare" a single controller against
  # itself and present the output as a result.
  if ! grep -q "^${CONTROLLER}," "$OUT_DIR/metrics.csv" 2>/dev/null; then
    echo >&2
    echo "error: $CONTROLLER recorded no metrics for $SCENARIO." >&2
    echo "       The comparison is incomplete and no results will be written." >&2
    echo "       Usual cause: the robot did not spawn, so /ground_truth/odom" >&2
    echo "       never published. Check the spawn_entity output above." >&2
    exit 1
  fi
done

echo
echo "== Analysis =="
python3 analysis/compare_controllers.py \
  --metrics "$OUT_DIR/metrics.csv" \
  --timeseries "$OUT_DIR/timeseries" \
  --figures analysis/figures/gazebo \
  --results docs/RESULTS_GAZEBO.md \
  --reference-speed "$SPEED"

echo
echo "Done. See docs/RESULTS_GAZEBO.md"
