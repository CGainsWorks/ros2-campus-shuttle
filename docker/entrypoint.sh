#!/usr/bin/env bash
# Source ROS and the built workspace, then run whatever was asked for.
#
# The workspace is built at /opt/shuttle_ws, deliberately outside /ws: the
# repository is mounted at /ws so results reach the host and that mount would
# otherwise hide install/.
set -e

# The image sets ROS_DISTRO without sourcing anything, so source explicitly.
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /opt/shuttle_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/shuttle_ws/install/setup.bash
fi

exec "$@"
