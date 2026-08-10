#!/usr/bin/env bash
#
# Smoke test for the ROS 2 side of the workspace: the things that compile fine
# but fail at runtime -- plugin registration, xacro expansion, message
# generation, launch file syntax.
set -uo pipefail

failed=0
note() { echo; echo "== $1 =="; }

note "Executables"
ros2 pkg executables shuttle_control || failed=1
ros2 pkg executables shuttle_localisation || failed=1
ros2 pkg executables shuttle_perception || failed=1

note "Message types"
# `ros2 interface package` reads the ament index directly. `ros2 interface list`
# goes through the daemon, which returns nothing on a cold start and makes this
# check spuriously fail.
if ros2 interface package shuttle_msgs 2>/dev/null | grep -q "TrackingError"; then
  ros2 interface package shuttle_msgs
else
  echo "FAIL: shuttle_msgs interfaces not found"
  failed=1
fi

note "URDF expands"
urdf_path="$(ros2 pkg prefix shuttle_description)/share/shuttle_description/urdf/shuttle.urdf.xacro"
if xacro "$urdf_path" > /tmp/robot.urdf 2>/tmp/xacro.err; then
  echo "ok: $(grep -c '<link' /tmp/robot.urdf) links, $(grep -c '<joint' /tmp/robot.urdf) joints"
else
  echo "FAIL: xacro could not expand the URDF"
  cat /tmp/xacro.err
  failed=1
fi

note "Nav2 controller plugins are registered"
for plugin in shuttle_control/PurePursuitController shuttle_control/StanleyController; do
  if grep -q "$plugin" \
      "$(ros2 pkg prefix shuttle_control)/share/shuttle_control/plugins.xml"; then
    echo "ok: $plugin"
  else
    echo "FAIL: $plugin not declared"
    failed=1
  fi
done

note "Launch files parse"
for launch in sim.launch.py slam.launch.py nav2.launch.py comparison.launch.py; do
  path="$(ros2 pkg prefix shuttle_bringup)/share/shuttle_bringup/launch/$launch"
  if python3 -c "
import sys
sys.argv = ['$launch']
from launch import LaunchDescription
import importlib.util
spec = importlib.util.spec_from_file_location('m', '$path')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
ld = m.generate_launch_description()
assert isinstance(ld, LaunchDescription)
print('ok: $launch (%d actions)' % len(ld.entities))
" 2>/tmp/launch.err; then
    :
  else
    echo "FAIL: $launch"
    tail -5 /tmp/launch.err
    failed=1
  fi
done

note "Scenario path publisher imports and builds every scenario"
python3 - <<'PY' || failed=1
from shuttle_localisation.scenario_paths import SCENARIOS
for name, fn in sorted(SCENARIOS.items()):
    pts, start = fn()
    print(f"ok: {name}: {len(pts)} points, start {start}")
PY

echo
if [ "$failed" -ne 0 ]; then
  echo "ROS smoke test FAILED"
  exit 1
fi
echo "ROS smoke test passed."
