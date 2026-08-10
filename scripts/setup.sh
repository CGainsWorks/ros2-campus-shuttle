#!/usr/bin/env bash
#
# Set up a native (non-Docker) development environment on Ubuntu 22.04.
#
# Docker is the supported path -- `docker compose up` reproduces the headline
# result without any of this. Use this only if you want to build the ROS 2
# workspace directly on the host.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ ! -f /etc/os-release ] || ! grep -q "22.04" /etc/os-release; then
  echo "warning: this script targets Ubuntu 22.04 (ROS 2 Humble)." >&2
  echo "         On anything else, use Docker: docker compose up" >&2
fi

echo "== Toolchain and Python dependencies =="
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git \
  python3-pip python3-numpy python3-scipy python3-matplotlib python3-pandas \
  python3-colcon-common-extensions python3-rosdep

if ! command -v ros2 >/dev/null 2>&1; then
  cat >&2 <<'EOF'

ROS 2 Humble was not found. Install it first:
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

The controller comparison does NOT need ROS. To run just that:
  cmake -S src/shuttle_control/core -B build/core -DCMAKE_BUILD_TYPE=Release
  cmake --build build/core -j
  ./build/core/test_core && ./build/core/controller_bench --out results/bench --tune

EOF
  exit 0
fi

echo
echo "== ROS dependencies =="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble

echo
echo "== Building the workspace =="
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cat <<'EOF'

Done. Source the workspace before using it:

  source install/setup.bash

Then:
  scripts/run_bench.sh                        the headline comparison
  scripts/verify_ros.sh                       ROS smoke test
  ros2 launch shuttle_bringup nav2.launch.py  full autonomy in Gazebo
EOF
