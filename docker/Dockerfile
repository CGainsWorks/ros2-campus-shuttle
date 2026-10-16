# Full simulation stack: ROS 2 Jazzy, modern Gazebo (gz-sim Harmonic) through
# ros_gz, Nav2, slam_toolbox and robot_localization.
#
# Jazzy and Harmonic are the officially paired releases: the Jazzy ros_gz
# binaries are built against Harmonic, which arrives through the gz_*_vendor
# packages, so no second package repository is involved. (Humble's pairing,
# Fortress, reached end of life in September 2026.)
#
# This image is large. The headline controller comparison does not need it;
# that runs in docker/Dockerfile.bench, which builds in under a minute. This
# image is for driving the robot in simulation, the Gazebo cross-check and the
# integration tests and it is the image CI runs those tests in.
#
# Base pinned by digest as well as tag, so a rebuild months from now starts
# from the same ROS packages that were tested.
FROM ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca

# Space-separated rosdep keys to leave out. CI passes "rviz2" because nothing
# headless needs a 3D viewer; the default image keeps it for interactive use.
ARG ROSDEP_SKIP_KEYS=""

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    MPLBACKEND=Agg

# hadolint ignore=DL3008
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-numpy \
      python3-scipy \
      python3-matplotlib \
      python3-pandas \
      python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# The workspace is built at /opt/shuttle_ws, NOT at the working directory.
#
# docker-compose mounts the repository at /ws so results land on the host. If
# the workspace were built at /ws, that mount would shadow install/ and every
# `ros2 launch` would fail with "package not found". Keeping the built workspace
# outside the mount point avoids that entirely.
WORKDIR /opt/shuttle_ws

# Dependencies first, resolved by rosdep from the package manifests alone. This
# is the slow layer and copying only the manifests means it is rebuilt (and
# CI's layer cache invalidated) only when a dependency actually changes.
COPY src/shuttle_msgs/package.xml src/shuttle_msgs/package.xml
COPY src/shuttle_description/package.xml src/shuttle_description/package.xml
COPY src/shuttle_control/package.xml src/shuttle_control/package.xml
COPY src/shuttle_localisation/package.xml src/shuttle_localisation/package.xml
COPY src/shuttle_perception/package.xml src/shuttle_perception/package.xml
COPY src/shuttle_bringup/package.xml src/shuttle_bringup/package.xml
# hadolint ignore=DL3008
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
         --skip-keys "${ROSDEP_SKIP_KEYS}" \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/shuttle_ws/

# Build the workspace, tests included, so `colcon test` runs in this image. The
# controller core is compiled as part of shuttle_control, so the Nav2 plugins,
# the nodes and the offline bench share objects.
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build \
         --symlink-install \
         --event-handlers console_direct- \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

# Where the mounted repository appears at run time.
WORKDIR /ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
