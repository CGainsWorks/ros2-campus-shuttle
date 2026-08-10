# Full simulation stack: ROS 2 Humble, Gazebo Classic 11, Nav2, slam_toolbox
# and robot_localization.
#
# This image is large and slow to build. The headline controller comparison does
# not need it -- that runs in docker/Dockerfile.bench, which builds in under a
# minute -- so anyone reproducing the numbers can skip this entirely. This image
# is for driving the robot in simulation and for the Gazebo cross-check.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONDONTWRITEBYTECODE=1 \
    MPLBACKEND=Agg

# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      python3-numpy \
      python3-scipy \
      python3-matplotlib \
      python3-pandas \
      python3-opencv \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-plugins \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-slam-toolbox \
      ros-humble-robot-localization \
      ros-humble-tf-transformations \
      ros-humble-teleop-twist-keyboard \
      ros-humble-cv-bridge \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# The workspace is built at /opt/shuttle_ws, NOT at the working directory.
#
# docker-compose mounts the repository at /ws so results land on the host. If
# the workspace were built at /ws, that mount would shadow install/ and every
# `ros2 launch` would fail with "package not found". Keeping the built workspace
# outside the mount point avoids that entirely.
WORKDIR /opt/shuttle_ws
COPY . /opt/shuttle_ws/

# Build the workspace. The controller core is compiled as part of
# shuttle_control, so the Nav2 plugins and the offline bench share objects.
RUN . /opt/ros/humble/setup.sh \
    && colcon build \
         --symlink-install \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build/*/CMakeFiles

# Where the mounted repository appears at run time.
WORKDIR /ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
