# ros2-campus-shuttle

> **Status: planned. This repository is a brief, not an implementation.**
> Nothing here is built yet. The sections below define what goes in it.

A complete ROS2 autonomy stack for a differential-drive campus shuttle, running on an NVIDIA Jetson Orin. The headline contribution is a like-for-like comparison of Pure Pursuit against Stanley path tracking on the same platform, with quantified cross-track and heading error.

---

## What this covers

CV entries this repository is the evidence for:

- Autonomous Campus Shuttle Robot (BEng final-year project)

---

## Requirements

### Hardware

- NVIDIA Jetson Orin
- Differential-drive or Ackermann chassis
- 2D or 3D LiDAR
- IMU
- Wheel encoders
- RGB camera
- Battery and power distribution
- Physical emergency-stop switch

### Software

- Ubuntu 22.04 and ROS2 Humble
- SLAM Toolbox or GMapping, plus AMCL
- robot_localization EKF
- Nav2
- Pure Pursuit and Stanley controllers in C++
- Camera-to-LiDAR extrinsic calibration
- Gazebo and MuJoCo
- RViz2

### Report and documentation deliverables

- Dissertation PDF
- Controller comparison with tracking-error plots
- Calibration procedure with residuals
- URDF and TF tree diagram
- Safety case
- Demo video

---

## Inputs

- LiDAR scans
- IMU readings
- Wheel odometry
- Camera frames
- Goal poses
- A prior map, or an unmapped environment for SLAM
- Calibration target images

## Outputs

- Occupancy grid map
- Localised robot pose with covariance
- Global and local plans
- Velocity commands on /cmd_vel
- Cross-track and heading error metrics for both controllers
- Rosbag recordings
- Calibration YAML

---

## Intended structure

```
src/
  shuttle_bringup/        launch files and all YAML params
  shuttle_description/    URDF, meshes, RViz configs
  shuttle_control/        Pure Pursuit and Stanley, C++
  shuttle_localisation/   EKF wrapper and TF publishers
  shuttle_perception/     camera-to-LiDAR calibration
  shuttle_msgs/
simulation/               Gazebo worlds and models, MuJoCo scenes
calibration/              intrinsics, extrinsics, procedure.md
data/                     maps, rosbag pointers (not the bags themselves)
analysis/                 notebooks and figures for the controller comparison
docker/
docs/
```

Plus the standard baseline: `README.md`, `LICENSE`, `.gitignore`, `.gitattributes`
(Git LFS rules for weights, bags and media), `.github/workflows/ci.yml`, `docs/`,
`scripts/setup.sh`, `scripts/run_demo.sh` and `tests/`.

---

## Build checklist

- [ ] Simulation bringup: URDF, Gazebo world, teleop drives the robot
- [ ] SLAM produces a usable map of the test environment
- [ ] EKF fusing LiDAR, IMU and wheel odometry, validated against ground truth
- [ ] Nav2 navigating to goals in simulation
- [ ] Pure Pursuit implemented and tuned
- [ ] Stanley implemented and tuned
- [ ] Automated comparison run producing error plots from the same rosbags
- [ ] Camera-to-LiDAR calibration with reported residual
- [ ] Hardware bringup on the Jetson Orin
- [ ] Demo video and results write-up

---

## Definition of done

This repository is finished when:

- [ ] The whole thing runs from one command on a clean machine.
- [ ] The README opens with a diagram or a GIF, not a wall of text.
- [ ] Every number quoted in the README is reproducible from a script in the repo.
- [ ] Limitations are stated honestly in their own section.
- [ ] A licence is declared.
- [ ] Large binaries live in Releases or LFS, not in the Git history.
- [ ] The repository URL is added to the matching CV bullet point.

---

## Starter prompt

Paste this into your coding agent of choice to begin.

<details>
<summary>Expand</summary>

```
You are helping me build a portfolio project from scratch. Read this brief and then
propose a concrete implementation plan before writing any code.

PROJECT: ros2-campus-shuttle
A complete ROS2 autonomy stack for a differential-drive campus shuttle, running on an NVIDIA Jetson Orin. The headline contribution is a like-for-like comparison of Pure Pursuit against Stanley path tracking on the same platform, with quantified cross-track and heading error.

TARGET STACK
- Ubuntu 22.04 and ROS2 Humble
- SLAM Toolbox or GMapping, plus AMCL
- robot_localization EKF
- Nav2
- Pure Pursuit and Stanley controllers in C++
- Camera-to-LiDAR extrinsic calibration
- Gazebo and MuJoCo
- RViz2

HARDWARE CONTEXT
- NVIDIA Jetson Orin
- Differential-drive or Ackermann chassis
- 2D or 3D LiDAR
- IMU
- Wheel encoders
- RGB camera
- Battery and power distribution
- Physical emergency-stop switch

THE SYSTEM MUST TAKE THESE INPUTS
- LiDAR scans
- IMU readings
- Wheel odometry
- Camera frames
- Goal poses
- A prior map, or an unmapped environment for SLAM
- Calibration target images

AND PRODUCE THESE OUTPUTS
- Occupancy grid map
- Localised robot pose with covariance
- Global and local plans
- Velocity commands on /cmd_vel
- Cross-track and heading error metrics for both controllers
- Rosbag recordings
- Calibration YAML

INTENDED REPOSITORY LAYOUT
src/
  shuttle_bringup/        launch files and all YAML params
  shuttle_description/    URDF, meshes, RViz configs
  shuttle_control/        Pure Pursuit and Stanley, C++
  shuttle_localisation/   EKF wrapper and TF publishers
  shuttle_perception/     camera-to-LiDAR calibration
  shuttle_msgs/
simulation/               Gazebo worlds and models, MuJoCo scenes
calibration/              intrinsics, extrinsics, procedure.md
data/                     maps, rosbag pointers (not the bags themselves)
analysis/                 notebooks and figures for the controller comparison
docker/
docs/

CONSTRAINTS
- Everything must run from a single command: `docker compose up` or `make demo`.
- No placeholder or stub code presented as working. If something is not implemented, say so.
- Every quantitative claim in the README must be reproducible from a script in this repo.
- Use only public or synthetic data. Record the licence of every dataset used.


START BY
1. Confirming or challenging the proposed repository layout.
2. Listing the specific libraries and versions you intend to pin.
3. Identifying the single riskiest technical unknown and how to de-risk it first.
Then work through the milestones in the README build checklist, one at a time.

```

</details>

---

## Notes

- Do not backdate commits to match CV dates. Publishing older work today is normal and expected.
- Record dataset licences before training on anything. Several common research datasets are
  non-commercial only.
- Flip this repository to public once there is something in it worth showing.
