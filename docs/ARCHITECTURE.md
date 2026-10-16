# Architecture

## Why the control code has no ROS dependency

Everything under `src/shuttle_control/core/` compiles against the C++ standard
library and nothing else. That one decision drives most of the repository's
shape.

It means the controller comparison can run headless, deterministically and in
about twenty seconds in a small Debian image, with no ROS installation and no
Gazebo. Anyone can reproduce the headline numbers. It also means the code under
comparison is provably the same in three places:

```
                    shuttle_control_core
                   (Path, SpeedLimiter,
                PurePursuit, Stanley, metrics)
                             |
        +--------------------+--------------------+
        |                    |                    |
  path_tracker_node    Nav2 plugins        controller_bench
  (ROS 2 node)      (nav2_core::Controller)  (offline, no ROS)
```

If the benchmark says Stanley overshoots on tight corners, that statement is
about the same object file Nav2 loads.

## Package layout

| Package | Contents |
| --- | --- |
| `shuttle_msgs` | `TrackingError`, `ControllerMetrics` |
| `shuttle_description` | URDF/xacro, Gazebo (gz-sim) systems and sensors, RViz configs |
| `shuttle_control` | The core library, the standalone tracker node, the Nav2 plugins, the safety gate, the metrics recorder, the benchmark |
| `shuttle_localisation` | EKF ground-truth evaluator, fixed reference-path publisher |
| `shuttle_perception` | Camera-to-LiDAR extrinsic calibration |
| `shuttle_bringup` | Launch files, all YAML parameters (including the ros_gz_bridge topic list), simulation assets, the launch_testing integration tests |

## Frames

```
map
 └── odom                      (global EKF, or the static spawn pose in comparison runs)
      └── base_footprint       (local EKF: wheel odometry + IMU)
           └── base_link
                ├── left_wheel_link / right_wheel_link
                ├── caster_link
                ├── lidar_link
                ├── imu_link
                └── camera_link
                     └── camera_optical_frame   (REP 103: z forward, x right, y down)
```

Exactly one node owns each transform. The Gazebo differential-drive system
publishes its transform on a model-scoped Gazebo topic that `ros_gz_bridge` is
deliberately not configured to carry and AMCL has `tf_broadcast` disabled,
because `robot_localization` owns `odom -> base_footprint` and `map -> odom`
respectively. Two publishers for one edge is the most common way a ROS TF tree
goes subtly wrong: it does not error, the transform just flickers between two
answers.

## Localisation

Two `robot_localization` EKF instances, which is the pattern the package is
designed for:

- **local** (`ekf.yaml`), `world_frame: odom`: fuses wheel odometry velocities
  and IMU yaw rate. Smooth and continuous, drifts without bound. This is what
  the controller runs on.
- **global** (`ekf_global.yaml`), `world_frame: map`: adds the AMCL pose.
  Absorbs discrete localisation corrections so they never make `odom` jump.

Wheel odometry contributes **velocities, not absolute pose**. Feeding integrated
dead-reckoned position into the filter would shrink the covariance around an
estimate that is drifting, which is worse than not fusing it at all.

The "LiDAR" in "EKF fusing LiDAR, IMU and wheel odometry" enters as the AMCL
pose (a scan match against the map) rather than as raw scans.

## Simulation

The simulator is modern Gazebo (gz-sim, **Harmonic**) with ROS 2 **Jazzy**,
the officially paired releases: the Jazzy `ros_gz` binaries are built against
Harmonic, which installs through the `gz_*_vendor` packages with no second
package repository. Gazebo Classic reached end of life in January 2025 and
Humble's pairing, Fortress, in September 2026, so moving to Jazzy and Harmonic
rather than Humble and Fortress is the only combination with years of support
left (Harmonic to 2028, Jazzy to 2029).

| | Gazebo Classic 11 (before) | gz-sim Harmonic (now) |
| --- | --- | --- |
| Differential drive | `libgazebo_ros_diff_drive.so`, odometry from the world pose | `gz-sim-diff-drive-system`, odometry from the wheel joints |
| Acceleration limit | `max_wheel_acceleration` 4.0 (rim speed) | 4.0 m/s² linear, 15.4 rad/s² angular (the same limit) |
| Ground truth | `libgazebo_ros_p3d.so`, 50 Hz | `gz-sim-odometry-publisher-system`, 50 Hz, world frame |
| LiDAR | `ray`, 720 samples, 0.16 to 12 m, σ 0.012 m, 10 Hz | `gpu_lidar`, identical specification |
| IMU | 100 Hz, gyro σ 0.002 rad/s, accel σ 0.017 m/s² | identical specification, `gz-sim-imu-system` |
| Camera | 1280 x 720, 1.089 rad FOV, 15 Hz, `camera_info` | identical specification, `camera_info` on `/camera/camera_info` |
| Physics | ODE, 2 ms step, real-time factor 1 | DART (the gz-sim default), 2 ms step, real-time factor 1 |
| ROS connection | plugins publish ROS topics directly | `ros_gz_bridge`, topic list in `config/gz_bridge.yaml` |
| Spawning | `spawn_entity.py` | `ros_gz_sim create` |

The robot description is the same xacro; only `shuttle.gazebo.xacro`, which
holds the simulator-specific elements, changed. Two differences are real rather
than cosmetic and are worth knowing when comparing old and new numbers: the
wheel odometry is now integrated from the wheel joints, as an encoder would
give it, where the Classic plugin was configured to report the world pose; and
the physics engine is DART rather than ODE.

The LiDAR and camera render with OGRE 2. The server runs with
`--headless-rendering`, which gives them an offscreen EGL context, so the full
sensor set works in a container or CI runner with no display and no GPU
(Mesa's software renderer).

## Safety layer

Every velocity source (the comparison path tracker, Nav2's velocity smoother,
teleop) publishes on `/cmd_vel_in`. Only `safety_gate_node` publishes
`/cmd_vel`, the topic the base listens to. The gate does two things:

- **command timeout**: if no command has arrived for `cmd_timeout` (0.5 s of
  simulated time by default) it holds the base at zero. The simulated
  differential drive, like a motor controller with no firmware timeout, would
  otherwise keep executing the last command for ever.
- **latched e-stop**: `/e_stop` (`std_msgs/Bool`) or the `/set_e_stop`
  service (`std_srvs/SetBool`) engages it. A zero command is published at once,
  every incoming command is dropped while it is held and `/e_stop/active`
  reports the state. Releasing it does not resume the last command; motion
  restarts only on a fresh one.

The decisions are made by `CommandGate` in the ROS-free core and unit tested on
synthetic time in `test_core`. `test_safety_sim.py` proves the behaviour against
the simulated base; the stop times it measures are in
[SAFETY.md](SAFETY.md).

## How the comparison run works

The comparison runs in `simulation/worlds/empty_test.sdf`, not the furnished
campus world. This is deliberate. The benchmark routes are laid out in their own
coordinates and several of them pass straight through the campus buildings (the
`corner` route's final leg runs through the library), so the robot drives into a
wall and the run records a stall that says nothing whatever about the control
law. Obstacle avoidance is the Nav2 demo's job; the comparison measures path
tracking and nothing else.

`scripts/run_comparison.sh` launches Gazebo once per controller. Each run:

1. `path_publisher` emits the scenario path, **latched**, on `/plan`. The path
   is generated from fixed geometry rather than planned by Nav2, because a
   replanned route can differ between runs and any difference in the reference
   confounds the difference between the controllers.
2. `path_tracker_node` follows it and publishes `/tracking_error`.
3. `metrics_recorder_node` scores the run against `/ground_truth/odom` (the
   exact model pose from Gazebo's odometry publisher system) and writes CSV in
   the same format the offline benchmark emits.
4. `analysis/compare_controllers.py` reads either source.

The recorder is C++ linking the core library rather than a Python node with its
own geometry, so Gazebo runs and offline runs are scored by literally the same
`Path::project()` and `summarise()`.

## Where the duplication is and how it is controlled

The scenario geometry exists twice: in C++ (`scenarios.cpp`, for the benchmark,
which must build without ROS) and in Python (`scenario_paths.py`, for the Gazebo
path publisher, which must run without a compiler).

Duplicated geometry drifts silently and a comparison run against a subtly
different reference path is worthless. `tests/test_scenarios_match.py` dumps the
C++ paths and checks the Python ones match point for point; they currently agree
to under a micrometre.

## Testing

| Test | What it protects |
| --- | --- |
| `test_core` (C++, 175 checks) | Path geometry, projection signs, speed limiter, plant integration, steering-sign conventions, determinism, **path continuity**, **speed survives a replan**, the safety gate's timeout and e-stop logic |
| `tests/test_scenarios_match.py` | The two scenario implementations agree |
| `tests/test_param_files.py` | Every ROS 2 parameter file will actually load |
| `tests/test_calibration.py` | Calibration accuracy, residual sits at the noise floor, degeneracy is detected |
| `scripts/run_all.sh` | Every expected artifact exists, is non-empty and every run finished cleanly |
| `scripts/verify_ros.sh` | Plugin registration, URDF expansion, message generation, launch parsing, URDF-to-SDF conversion, world validity |
| `test_safety_sim.py` (launch_testing) | In Gazebo: the e-stop and the command timeout each bring the base to rest within a stated time |
| `test_comparison_sim.py` (launch_testing) | In Gazebo: both controllers drive the `corner` route to the goal and the metrics are produced |
| ament linters (`colcon test`) | cpplint, uncrustify, cppcheck, xmllint, lint_cmake, copyright on the C++ packages |
| `scripts/check_readme.py` | Every number the README quotes matches `results/controller_comparison.json` and the calibration report |

Most of these exist because of a bug they caught.

- **Path continuity**: an early staircase generator emitted a path that jumped
  backwards 1.15 m at one corner. Both controllers stalled there and it looked
  exactly like a controller defect.
- **Parameter files**: the EKF covariance matrices mixed `0` and `0.02`. ROS 2
  rejects a parameter sequence that is not homogeneous, so the whole file failed
  to load, `ekf_node` died at startup and the only visible symptom was a
  disconnected TF tree.
- **Speed survives a replan**: `setPath()` used to reset the speed limiter, so
  every republished plan ramped the robot up from zero again. Only visible under
  Gazebo, where plans are republished; the benchmark installs a path once.
- **Artifact verification**: a pipeline that runs to completion is not the same
  as one that produced the results it claims. `run_all.sh` checks the files
  exist and are non-empty rather than trusting the console output.

## What running the real simulator caught

Three defects were structurally invisible to the offline benchmark and only
appeared under Gazebo (Classic, at the time):

1. A **spawn race**: `spawn_entity` gave up after its default 30 s while
   gzserver was still starting cold, so the robot never appeared. Everything
   downstream ran happily against an empty world and the run produced no data,
   silently dropping one controller from the comparison.
2. A **wall-clock timer under simulated time**: `create_wall_timer` does not
   follow the sim clock, so the control rate no longer matched the rate the
   robot experienced.
3. The **`setPath` speed reset** above, which made Pure Pursuit look like the
   worse controller for reasons unconnected to its control law: its control
   effort was 10.5 rad/s² before the fix and 0.44 rad/s² after.

This is the argument for keeping the heavyweight simulation around even though
the headline numbers come from the fast benchmark.
