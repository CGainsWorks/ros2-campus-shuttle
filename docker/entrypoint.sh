#!/usr/bin/env bash
# Source ROS and the built workspace, then run whatever was asked for.
#
# The workspace is built at /opt/shuttle_ws, deliberately outside /ws: the
# repository is mounted at /ws so results reach the host, and that mount would
# otherwise hide install/.
set -e

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /opt/shuttle_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/shuttle_ws/install/setup.bash
fi

# Gazebo needs to resolve the world's model and resource references.
export GAZEBO_MODEL_PATH="/opt/shuttle_ws/simulation/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="/opt/shuttle_ws/simulation:${GAZEBO_RESOURCE_PATH:-}"

exec "$@"
