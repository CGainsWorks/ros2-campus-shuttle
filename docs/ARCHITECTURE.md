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
| `shuttle_description` | URDF/xacro, Gazebo sensor plugins, RViz configs |
| `shuttle_control` | The core library, the standalone tracker node, the Nav2 plugins, the metrics recorder, the benchmark |
| `shuttle_localisation` | EKF ground-truth evaluator, fixed reference-path publisher |
| `shuttle_perception` | Camera-to-LiDAR extrinsic calibration |
| `shuttle_bringup` | Launch files, all YAML parameters, simulation assets |

## Frames

```
map
 └── odom                      (global EKF, or a static identity in comparison runs)
      └── base_footprint       (local EKF: wheel odometry + IMU)
           └── base_link
                ├── left_wheel_link / right_wheel_link
                ├── caster_link
                ├── lidar_link
                ├── imu_link
                └── camera_link
                     └── camera_optical_frame   (REP 103: z forward, x right, y down)
```

Exactly one node owns each transform. The Gazebo differential-drive plugin has
`publish_odom_tf` disabled and AMCL has `tf_broadcast` disabled, because
`robot_localization` owns `odom -> base_footprint` and `map -> odom`
respectively. Two publishers for one edge is the most common way a ROS TF tree
goes subtly wrong: it does not error, the transform just flickers between two
answers.

## Localisation

Two `robot_localization` EKF instances, which is the pattern the package is
designed for:

- **local** (`ekf.yaml`), `world_frame: odom` — fuses wheel odometry velocities
  and IMU yaw rate. Smooth and continuous, drifts without bound. This is what
  the controller runs on.
- **global** (`ekf_global.yaml`), `world_frame: map` — adds the AMCL pose.
  Absorbs discrete localisation corrections so they never make `odom` jump.

Wheel odometry contributes **velocities, not absolute pose**. Feeding integrated
dead-reckoned position into the filter would shrink the covariance around an
estimate that is drifting, which is worse than not fusing it at all.

The "LiDAR" in "EKF fusing LiDAR, IMU and wheel odometry" enters as the AMCL
pose — a scan match against the map — rather than as raw scans.

## How the comparison run works

The comparison runs in `simulation/worlds/empty_test.world`, not the furnished
campus world. This is deliberate. The benchmark routes are laid out in their own
coordinates and several of them pass straight through the campus buildings — the
`corner` route's final leg runs through the library — so the robot drives into a
wall and the run records a stall that says nothing whatever about the control
law. Obstacle avoidance is the Nav2 demo's job; the comparison measures path
tracking and nothing else.

`scripts/run_comparison.sh` launches Gazebo once per controller. Each run:

1. `path_publisher` emits the scenario path, **latched**, on `/plan`. The path
   is generated from fixed geometry rather than planned by Nav2, because a
   replanned route can differ between runs and any difference in the reference
   confounds the difference between the controllers.
2. `path_tracker_node` follows it and publishes `/tracking_error`.
3. `metrics_recorder_node` scores the run against `/ground_truth/odom` — the
   Gazebo p3d plugin's exact model pose — and writes CSV in the same format the
   offline benchmark emits.
4. `analysis/compare_controllers.py` reads either source.

The recorder is C++ linking the core library rather than a Python node with its
own geometry, so Gazebo runs and offline runs are scored by literally the same
`Path::project()` and `summarise()`.

## Where the duplication is, and how it is controlled

The scenario geometry exists twice: in C++ (`scenarios.cpp`, for the benchmark,
which must build without ROS) and in Python (`scenario_paths.py`, for the Gazebo
path publisher, which must run without a compiler).

Duplicated geometry drifts silently, and a comparison run against a subtly
different reference path is worthless. `tests/test_scenarios_match.py` dumps the
C++ paths and checks the Python ones match point for point; they currently agree
to under a micrometre.

## Testing

| Test | What it protects |
| --- | --- |
| `test_core` (C++, 145 checks) | Path geometry, projection signs, speed limiter, plant integration, steering-sign conventions, determinism, **path continuity**, **speed survives a replan** |
| `tests/test_scenarios_match.py` | The two scenario implementations agree |
| `tests/test_param_files.py` | Every ROS 2 parameter file will actually load |
| `tests/test_calibration.py` | Calibration accuracy, residual sits at the noise floor, degeneracy is detected |
| `scripts/run_all.sh` | Every expected artifact exists, is non-empty, and every run finished cleanly |
| `scripts/verify_ros.sh` | Plugin registration, URDF expansion, message generation, launch parsing |

Most of these exist because of a bug they caught.

- **Path continuity**: an early staircase generator emitted a path that jumped
  backwards 1.15 m at one corner. Both controllers stalled there and it looked
  exactly like a controller defect.
- **Parameter files**: the EKF covariance matrices mixed `0` and `0.02`. ROS 2
  rejects a parameter sequence that is not homogeneous, so the whole file failed
  to load, `ekf_node` died at startup, and the only visible symptom was a
  disconnected TF tree.
- **Speed survives a replan**: `setPath()` used to reset the speed limiter, so
  every republished plan ramped the robot up from zero again. Only visible under
  Gazebo, where plans are republished; the benchmark installs a path once.
- **Artifact verification**: a pipeline that runs to completion is not the same
  as one that produced the results it claims. `run_all.sh` checks the files
  exist and are non-empty rather than trusting the console output.

## What running the real simulator caught

Three defects were structurally invisible to the offline benchmark and only
appeared under Gazebo:

1. A **spawn race** — `spawn_entity` gave up after its default 30 s while
   gzserver was still starting cold, so the robot never appeared. Everything
   downstream ran happily against an empty world and the run produced no data,
   silently dropping one controller from the comparison.
2. A **wall-clock timer under simulated time** — `create_wall_timer` does not
   follow the sim clock, so the control rate no longer matched the rate the
   robot experienced.
3. The **`setPath` speed reset** above, which made Pure Pursuit look like the
   worse controller for reasons unconnected to its control law: its control
   effort was 10.5 rad/s² before the fix and 0.44 rad/s² after.

This is the argument for keeping the heavyweight simulation around even though
the headline numbers come from the fast benchmark.
