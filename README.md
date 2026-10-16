# ros2-campus-shuttle

A ROS 2 autonomy stack for a differential-drive campus shuttle, in simulation:
ROS 2 Jazzy, modern Gazebo (gz-sim Harmonic), Nav2, SLAM Toolbox,
`robot_localization` and a software safety layer.

The point of this repository is not that the robot navigates. It is a
**measured, like-for-like comparison of Pure Pursuit against Stanley path
tracking**: same robot, same paths, same speeds, same disturbances, with
cross-track and heading error quantified against ground truth and an honest
conclusion about which is better and when.

![Tracking error by scenario](analysis/figures/rms_by_scenario.png)

## The result

Across 84 runs (7 scenarios × 3 speeds × noise on/off × 2 controllers), none of
which failed to finish:

| Scenario | Tuned on | Pure Pursuit RMS | Stanley RMS | Closer |
| --- | --- | ---: | ---: | :---: |
| `straight_offset` (1 m step) | yes | 0.2101 m | **0.2028 m** | Stanley |
| `heading_offset` (30° step) | held out | 0.0126 m | **0.0033 m** | Stanley |
| `circle` (R = 6 m) | yes | **0.0001 m** | 0.0073 m | Pure Pursuit |
| `corner` (1.5 m turns) | yes | **0.0037 m** | 0.0124 m | Pure Pursuit |
| `figure_eight` | held out | **0.0009 m** | 0.0114 m | Pure Pursuit |
| `campus_loop` (125 m) | yes | **0.0011 m** | 0.0054 m | Pure Pursuit |
| `sharp_turns` (1.2 m turns) | held out | **0.0059 m** | 0.0192 m | Pure Pursuit |

*Simulated (kinematic benchmark). At 1.0 m/s, no odometry noise, measured at
`base_link` against ground truth.*

**Neither controller wins outright and that is the finding.**

- **Pure Pursuit holds a line far better once on it.** On every curved route
  it is at least 3.2× tighter. On a constant-radius arc its commanded curvature
  equals `1/R` *exactly* (the geometry is not an approximation there), which is
  why `circle` comes out at 0.1 mm.
- **Stanley re-acquires a path more gracefully.** It carries a lower RMS error
  through both step scenarios and, more usefully, **gets onto the line without
  overshooting it**. Recovering from a 30° heading error it never leaves a 5 cm
  band, while Pure Pursuit swings 8.3 cm past the path before settling.
- **The two trade settling time against overshoot.** From a 1 m lateral offset
  at 1.5 m/s, Pure Pursuit is inside 5 cm after 1.07 m but overshoots by
  3.6 cm; Stanley takes 3.45 m to get inside that band but overshoots only
  0.9 cm. RMS alone hides this, so `docs/RESULTS.md` reports settling distance
  and overshoot separately.
- **Stanley overshoots at corner exits.** Its error peaks of 19.0 mm on
  `campus_loop` and 46.0 mm on `sharp_turns`, against Pure Pursuit's 5.5 mm and
  14.2 mm, are the ringing visible in the figure below. It regulates error at a
  virtual front axle, so it commits to the turn later and unwinds it harder.
- **Both degrade about the same with odometry noise** (+24.1% against +24.5%),
  so the usual claim that Stanley is the more heading-noise-sensitive of the two
  is not reproduced at these gains and speeds.

So: **Pure Pursuit for route following, Stanley if the robot is frequently
re-acquiring a path** (after a replan, a manual takeover or a localisation
jump) and particularly where overshooting the path is worse than taking longer
to reach it, which on a pedestrian campus it usually is.

![Trajectories and tracking error](analysis/figures/trajectories.png)

Full numbers, including the speed sweep and control effort, are in
**[docs/RESULTS.md](docs/RESULTS.md)**, generated from the raw CSV rather than
written by hand. Every figure quoted in this README is also recorded in
**[results/controller_comparison.json](results/controller_comparison.json)**
and CI fails if the README and that file disagree (`scripts/check_readme.py`).

## Reproducing it

```bash
docker compose up
```

That builds a small Debian image and runs the whole pipeline: the controller
unit tests, the scenario geometry cross-check, the parameter file validation,
the camera-to-LiDAR calibration, the gain tuning, all 84 comparison runs and
every figure and `docs/RESULTS.md`. It then **verifies** that each expected
artifact exists and is non-empty, that all 84 runs finished cleanly and that
there is one time series per run. No ROS, no Gazebo, no GPU.

Every benchmark number in this README comes out of that command. It ends with
`END-TO-END PASSED`, or a non-zero exit.

The benchmark is deterministic: running it on a fresh clone regenerates every
committed artifact (`docs/RESULTS.md`, all four figures, the tuned gains, the
calibration and the benchmark section of `results/controller_comparison.json`)
**byte for byte identical**, so `git status` comes back clean afterwards. CI
checks exactly that on every push. (The Gazebo cross-check is not
deterministic; see below.)

```bash
docker compose run --rm comparison    # just the controller comparison
docker compose run --rm test          # all checks that don't need ROS
docker compose run --rm calibration   # camera-to-LiDAR extrinsics
```

## How the comparison is kept fair

This is the part that matters and most of the design follows from it.

- **The longitudinal controller is shared code.** Both trackers pick speed with
  the same lateral-acceleration and deceleration-to-goal limits. If they also
  disagreed about speed, any difference in tracking error could be explained by
  one of them simply taking the corner slower.
- **Error is reported at `base_link` for both**, even though Stanley internally
  regulates error at a virtual front axle, a different and smaller quantity.
- **Error is measured against ground truth**, never against the pose the
  controller was given. A controller that tracks a drifting estimate beautifully
  should score badly and this is what makes that happen.
- **The noise seed is derived from the scenario, speed and noise setting, never
  from the controller**, so both face a bit-identical disturbance sequence.
- **Gains are fitted by grid search, not chosen by hand**, on a tuning subset
  and reported on all seven scenarios. `heading_offset`, `figure_eight` and
  `sharp_turns` are held out; `sharp_turns` uses corners tighter than anything
  in the tuning set.
- **A 100 ms sensing latency is modelled.** Without it the tuner drives the
  Pure Pursuit lookahead to 0.15 m, because in a latency-free world a shorter
  lookahead simply tracks better. Latency is what makes an unreasonably short
  lookahead oscillate.
- **The tuning objective penalises control effort**, not just error. Minimising
  error alone selects the most aggressive gains on the grid, right at the edge
  of oscillation.

## What is here

| | |
| --- | --- |
| **Controllers** | Pure Pursuit and Stanley in C++, as a standalone node **and** as `nav2_core::Controller` plugins |
| **Simulation** | Gazebo Harmonic (gz-sim) through `ros_gz`: campus and test worlds, URDF with 2D LiDAR, IMU, RGB camera and a ground-truth odometry publisher, all bridged with `ros_gz_bridge` |
| **Safety layer** | Command-timeout watchdog and latched e-stop (topic and service) between every command source and the base, proved in simulation by a launch test |
| **Localisation** | Two `robot_localization` EKF instances (local `odom`, global `map`), with a NEES-based consistency check against ground truth |
| **Navigation** | Nav2 with both controllers registered; SLAM Toolbox for mapping, AMCL for localisation |
| **Calibration** | Camera-to-LiDAR extrinsics by point-to-plane least squares, with observability checking |
| **Analysis** | Automated comparison harness, figures and results document generated from raw CSV |

### The core design decision

Everything in `src/shuttle_control/core/` compiles against the C++ standard
library and nothing else. The same object files are linked into the ROS 2 node,
the Nav2 plugins and the offline benchmark:

```
                    shuttle_control_core
                   (Path, SpeedLimiter,
             PurePursuit, Stanley, CommandGate,
                         metrics)
                             |
        +--------------------+--------------------+
        |                    |                    |
  path_tracker_node    Nav2 plugins        controller_bench
  safety_gate_node  (nav2_core::Controller)  (offline, no ROS)
   (ROS 2 nodes)
```

So the comparison runs in seconds on any machine and the code being compared is
provably the same code Nav2 loads. More in
**[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.

## Camera-to-LiDAR calibration

Solved as point-to-plane least squares: a checkerboard gives the board plane in
the camera frame, the LiDAR gives points on that same board and the extrinsic
is the transform that puts those points on that plane.

Against a known transform with 12 board poses and 1440 points (synthetic data):

| | |
| --- | --- |
| Residual (RMS point-to-plane) | **12.0 mm** |
| Translation error vs truth | 8.3 mm |
| Rotation error vs truth | 0.37° |
| Jacobian condition number | 116 |

The residual sits exactly at the injected 12 mm LiDAR noise floor, which is the
useful check: the estimator is noise-limited, not model-limited. The solver also
reports the condition number and warns when the board poses are close to
degenerate. Near-parallel boards leave rotation about the shared normal
unobservable and a solver that does not check will return a confident wrong
answer. `tests/test_calibration.py` verifies all of this.

Reproduce with `docker compose run --rm calibration`. Details and the
real-robot capture procedure: **[calibration/procedure.md](calibration/procedure.md)**.

## Running the simulation

The simulator is **Gazebo Harmonic** (gz-sim 8) with **ROS 2 Jazzy**, the
officially paired releases: the Jazzy `ros_gz` binaries are built against
Harmonic. Gazebo Classic, which this project used before, reached end of life
in January 2025 and Humble's pairing, Fortress, in September 2026, so Jazzy
with Harmonic is the supported combination (to 2029 and 2028). What changed in
the move, sensor by sensor, is in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#simulation).

The simulation image is a multi-gigabyte build and is **not** needed for the
benchmark results above.

```bash
# Interactive shell in the simulation image
docker compose --profile sim run --rm sim

# Then, inside:
ros2 launch shuttle_bringup slam.launch.py rviz:=true      # map the world
ros2 launch shuttle_bringup nav2.launch.py rviz:=true      # full autonomy
ros2 launch shuttle_bringup comparison.launch.py \
    controller:=stanley scenario:=corner x:=0.0            # one measured run

# Drive by hand, through the safety gate
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_in
```

The server runs headless with offscreen rendering, so the LiDAR and camera work
with no display and no GPU. Pass `headless:=false` for the Gazebo GUI.

The Gazebo cross-check runs the identical controller code against the physics
engine, scored against the Gazebo ground-truth pose and repeats it:

```bash
docker compose --profile sim run --rm gazebo-comparison
```

### What the Gazebo cross-check showed

On the `corner` route at 1.0 m/s, both controllers reach the goal every time.
Across 3 independent repeats (simulated, Gazebo Harmonic):

| RMS cross-track | Pure Pursuit | Stanley |
| --- | ---: | ---: |
| repeat 1 | 0.0566 m | 0.0479 m |
| repeat 2 | 0.0513 m | 0.1215 m |
| repeat 3 | 0.1114 m | 0.0583 m |

**Gazebo does not separate these two controllers on this route.** Stanley is
closer in repeats 1 and 3 and Pure Pursuit in repeat 2. Each controller's own
result more than doubles from its best repeat to its worst, which is more than
any gap between them, so no ranking survives. Each controller's worst repeat is
also the one where its localisation drifted most (below): the error follows the
odometry, not the control law. The spread depends on the machine too. On a
busier CPU the dead-reckoning drift grows and so does the measured error, which
is why the CI integration test bounds it only as a sanity limit. This is why
the headline comparison is the deterministic benchmark, where the same inputs
give bit-identical outputs and an 84-run matrix backs the conclusion.

Why the errors are an order of magnitude above the benchmark's: they are not
measuring the same thing. The comparison runs use a static `map -> odom` (the
spawn pose) with AMCL disabled, so the controller drives on dead-reckoned
odometry while error is scored against ground truth. The EKF evaluator
measures that drift directly over the same runs:

| EKF against Gazebo ground truth | across all repeats and controllers |
| --- | ---: |
| Position error (RMS) | 0.204 to 0.442 m |
| Position error (max) | 0.263 to 0.625 m |
| Heading error (RMS) | 0.0169 to 0.0415 rad |

**The localisation drift is larger than the entire measured tracking error.**
Split along and across the direction of travel, it is 0.20 to 0.42 m RMS along
it and 0.053 to 0.126 m across it. The along-track part only moves the point
where the robot believes the route ends; the across-track part moves the robot
off the line and on its own it is the size of the whole measured tracking
error (0.048 to 0.121 m) run by run. So the Gazebo figures are dominated by odometry rather
than by the control law, which is the other reason they cannot rank the two
controllers. Treat the bench numbers as the controller comparison and these as
a system-level check: the stack drives the route, the localisation drifts about
as much as you would expect from dead reckoning and nothing diverges.

These numbers replace the ones measured under Gazebo Classic. The two
simulators are not interchangeable: the wheel odometry is now integrated from
the wheel joints as an encoder would give it (the Classic plugin was configured
to report the world pose) and the physics engine is DART rather than ODE.

The evaluator also reports NEES for covariance consistency. Both its position
and heading NEES come out at about zero here, which is not a defect in the
filter: this instance fuses body-frame velocities and yaw rate only, so absolute
position *and* absolute heading are unobservable and every pose covariance grows
without bound. NEES then tends to zero simply because the reported uncertainty
is enormous; it reports tens of metres while being accurate to decimetres. The
evaluator prints that caveat alongside the number rather than leaving a reader
to interpret a suspicious zero. NEES becomes meaningful for the global filter,
where AMCL supplies an absolute fix.

Running the physics simulator is what surfaced three bugs the offline benchmark
structurally could not: a spawn race that silently dropped one controller from
the results, a wall-clock timer under simulated time and `setPath()`
discarding the robot's speed on every republished plan. That last one made
Pure Pursuit look like the worse controller for a reason that had nothing to do
with its control law: its control effort was 10.5 rad/s² before the fix and
0.44 rad/s² after. The move to gz-sim found a fourth: `gz sim` forks the real
server, so stopping a launch could leave a running world behind and the next
run in the same transport partition recorded a robot 17 m from where it was.
`shuttle_bringup/scripts/gz_server` now stops the whole server process group.

Which tracker Nav2 uses is a parameter, not a rebuild: both are registered as
plugins in `config/nav2_params.yaml` under the ids `FollowPath` and
`StanleyPath`.

## Safety layer

Every velocity source (the comparison tracker, Nav2's velocity smoother,
teleop) publishes on `/cmd_vel_in`; only `safety_gate_node` publishes
`/cmd_vel`, which is what the base executes. The gate has a 0.5 s command
timeout and a latched e-stop on the `/e_stop` topic (`std_msgs/Bool`) and the
`/set_e_stop` service (`std_srvs/SetBool`), with the state on `/e_stop/active`.
Commands that arrive during a stop are dropped and releasing it does not
resume them.

The stated requirement is that, from 1.0 m/s, the base is at rest (ground-truth
speed under 0.05 m/s) within 0.5 s of the e-stop and within 1.0 s of the last
command when commands stop. `src/shuttle_bringup/test/test_safety_sim.py`
proves both in Gazebo on every CI run. Measured (simulated time): from 1.00 m/s
to rest in 0.26 s after the e-stop, with full-speed commands still arriving;
and at rest 0.77 s after the last command for the timeout. Details and what is
still not covered: **[docs/SAFETY.md](docs/SAFETY.md)**.

## Tests

| Test | Covers |
| --- | --- |
| `test_core` (C++, 175 checks) | Path geometry, projection sign conventions, speed limiter, plant integration, steering signs, determinism, path continuity, speed survives a replan, the safety gate's timeout and e-stop logic |
| `tests/test_scenarios_match.py` | The C++ and Python scenario generators agree (currently to under a micrometre) |
| `tests/test_calibration.py` | Calibration accuracy, residual at the noise floor, degeneracy detection |
| `tests/test_param_files.py` | Every ROS 2 parameter file will actually load |
| `scripts/run_all.sh` | Every expected artifact exists, is non-empty and all 84 runs finished cleanly |
| `scripts/check_readme.py` | Every number this README quotes matches `results/controller_comparison.json` and the calibration report |
| `scripts/verify_ros.sh` | Plugin registration, URDF expansion and conversion to SDF, world validity, message generation, launch-file parsing |
| `test_safety_sim.py` (launch_testing, Gazebo) | E-stop and command timeout each bring the base to rest within the stated time |
| `test_comparison_sim.py` (launch_testing, Gazebo) | Simulator, stack and comparison scenario, headless: both controllers reach the goal on `corner` and the metrics are produced |
| ament linters (`colcon test`) | cpplint, uncrustify, cppcheck, xmllint, lint_cmake and copyright on the C++ packages |

CI runs all of it on every push: the benchmark job (about two minutes) and a
simulation job that builds `docker/Dockerfile` with its dependency layer cached
and runs `colcon test` in it, Gazebo included.

The path-continuity check exists for a reason. An early version of the
`sharp_turns` generator emitted a path that jumped backwards by 1.15 m at one
corner. Both controllers stalled there and it looked exactly like a controller
defect: the "finding" would have been that Pure Pursuit limit-cycles on tight
turns. It was a broken reference path. A comparison is only worth reporting if
the reference trajectories are sound, so they are now checked rather than
assumed.

`test_param_files.py` has a similar origin. The EKF covariance matrices were
written with a mix of `0` and `0.02`; ROS 2 requires a parameter sequence to be
homogeneous and rejected the entire file, so `ekf_node` died at startup and the
only visible symptom was a disconnected TF tree and a robot that never moved.
Nothing in a build catches that, so it is checked directly.

## Limitations

Stated plainly, because they bound what the numbers mean.

- **This is simulation only.** No hardware bringup, no Jetson Orin, no physical
  emergency stop. The BEng project this rebuilds ran on hardware; this does not.
- **The safety layer is software.** It stops the simulated base on an e-stop or
  a stale command but it shares the failure modes of the stack it guards: if
  the gate itself dies, the simulated base keeps its last command. A real base
  needs a firmware command timeout and an e-stop that cuts motor power in
  hardware. Neither is simulated. See [docs/SAFETY.md](docs/SAFETY.md).
- **Stanley on a differential drive requires an adaptation.** The published law
  is defined for a front-wheel-steered vehicle. Applying it here needs a
  *virtual front axle* distance `L_fa`, which acts as a wheelbase. It is a
  tuning parameter of the adaptation, not a property of the robot and a
  different `L_fa` moves Stanley's numbers. This is the largest caveat on the
  comparison; see [docs/CONTROLLERS.md](docs/CONTROLLERS.md).
- **The headline numbers come from a kinematic benchmark**, which models
  actuator lag, wheel saturation, sensing latency and odometry noise but not
  wheel slip, tyre dynamics or LiDAR mis-association. The Gazebo cross-check
  runs and is reported above but on one route at one speed and its error is
  dominated by odometry drift rather than by the control law, so it validates
  the system, not the controller ranking.
- **Absolute errors are small because the plant is well behaved.** The
  comparison between the two columns is the result; the magnitude of either is
  not a claim about a real vehicle.
- **The comparison runs deliberately bypass AMCL**, using the spawn pose for
  the map frame, so that localisation error does not contaminate the controller
  measurement. Real total path error would be larger.
- **The LiDAR is 2D.** One horizontal plane at 0.37 m above the ground. Fine for
  a supervised demonstration, not sufficient for operating unsupervised around
  people. See [docs/SAFETY.md](docs/SAFETY.md).
- **The EKF is validated only against dead reckoning, not against AMCL.** The
  evaluator works and its numbers are quoted above but the comparison runs
  disable AMCL, so what is measured is the local filter's drift. The global
  `map`-frame filter that fuses the AMCL pose is configured and launched by
  `nav2.launch.py` but its accuracy is not quantified here and no map is
  committed for it (`data/README.md` says how to make one).
- **Rendering in CI is software rendering.** The LiDAR and camera render with
  OGRE 2 on Mesa's CPU renderer, which is slower than a GPU but gives the same
  sensor model. The simulator's achieved real-time factor varies with the
  machine; every node runs on the simulation clock, so results do not depend on
  it.
- **MuJoCo is not implemented.** The original brief listed it alongside Gazebo;
  a second physics engine adds no evidence the comparison needs, so the effort
  went into the ROS-free benchmark instead.
- **No dissertation PDF or demo video** in this repository.

Current status, what is done and what is not: **[STATUS.md](STATUS.md)**.

## Repository layout

```
src/
  shuttle_msgs/           TrackingError, ControllerMetrics
  shuttle_description/    URDF, Gazebo (gz-sim) systems and sensors, RViz configs
  shuttle_control/        controllers and safety gate (core/ is ROS-free), nodes,
                          Nav2 plugins, bench
  shuttle_localisation/   EKF ground-truth evaluation, reference-path publisher
  shuttle_perception/     camera-to-LiDAR extrinsic calibration
  shuttle_bringup/        launch files, YAML parameters, ros_gz_bridge topics,
                          launch_testing integration tests
simulation/worlds/        Gazebo worlds (SDF): campus and the empty test world
analysis/                 comparison plots, results generator and summariser
calibration/              extrinsics and the capture procedure
results/                  controller_comparison.json (the rest is not committed)
docs/                     architecture, controllers, results, safety
scripts/                  run_bench.sh, run_comparison.sh, run_gazebo_repeats.sh,
                          run_calibration.sh, check_readme.py
tests/
```

## Documentation

- **[docs/RESULTS.md](docs/RESULTS.md)**: full comparison, generated from the data
- **[docs/RESULTS_GAZEBO.md](docs/RESULTS_GAZEBO.md)**: the Gazebo cross-check, generated the same way
- **[docs/CONTROLLERS.md](docs/CONTROLLERS.md)**: the maths, the diff-drive Stanley adaptation, tuning
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**: packages, frames, the simulator, the safety layer, how a run works
- **[docs/SAFETY.md](docs/SAFETY.md)**: hazards, mitigations, measured stop times, what is not simulated
- **[calibration/procedure.md](calibration/procedure.md)**: calibration method and capture procedure
- **[STATUS.md](STATUS.md)**: complete, in progress, not started, limitations

## Licence

MIT; see [LICENSE](LICENSE).
