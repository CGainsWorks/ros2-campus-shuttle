# Status

Last updated 2026-10-16. Everything here runs in simulation; nothing has been
run on hardware.

## Complete

- **Controller comparison (kinematic benchmark).** Pure Pursuit and Stanley in
  a ROS-free C++ core, 84 deterministic runs (7 scenarios × 3 speeds × noise
  on/off × 2 controllers), gains fitted by grid search with held-out scenarios.
  `docs/RESULTS.md`, the figures and the benchmark section of
  `results/controller_comparison.json` regenerate byte for byte and CI checks
  that on every push.
- **README checked against the data.** `scripts/check_readme.py` compares every
  number the README quotes (benchmark, Gazebo cross-check, safety stop times,
  calibration, unit test count) with `results/controller_comparison.json` and
  the calibration report: 61 figures plus the test count, run in CI.
- **Simulation on modern Gazebo.** Migrated from Gazebo Classic 11 (end of life
  January 2025) to Gazebo Harmonic (gz-sim 8.15.0) with ROS 2 Jazzy, the
  officially paired releases. `ros_gz_bridge` carries LiDAR, IMU, camera and
  camera_info, wheel odometry, ground truth, joint states, clock and cmd_vel;
  robot, worlds and sensor specifications are carried over unchanged
  (`docs/ARCHITECTURE.md#simulation` lists every element). Headless, with
  offscreen rendering for the LiDAR and camera.
- **Safety layer.** `safety_gate_node` with a 0.5 s command timeout and a
  latched e-stop (topic and service) in front of the base, logic unit tested in
  the core and `test_safety_sim.py` proving in Gazebo that the base is at rest
  within 0.5 s of an e-stop and within 1.0 s of the last command (simulated:
  0.26 s and 0.77 s measured).
- **Headless integration test in CI.** `test_comparison_sim.py` starts Gazebo,
  the bridge, the gate, the EKF, the path publisher, the tracker and the
  metrics recorder, runs `corner` at 1.0 m/s with each controller and asserts
  the goal is reached and the metrics are produced.
- **Gazebo cross-check.** Three repeats of `corner` at 1.0 m/s with both
  controllers, every repeat recorded in `results/controller_comparison.json`.
  Gazebo does not separate the controllers on this route (the gap is smaller
  than the spread between repeats), which the README states.
- **Localisation.** Local and global `robot_localization` EKFs; the evaluator
  measures the local filter's drift against Gazebo ground truth.
- **Nav2.** Both controllers registered as `nav2_core::Controller` plugins;
  SLAM Toolbox and AMCL configured.
- **Camera-to-LiDAR calibration** on synthetic data, with degeneracy detection.
- **CI.** Two jobs: the benchmark, results and README job and the simulation
  job, which builds `docker/Dockerfile` (dependency layer cached in the Actions
  cache) and runs `colcon test`: ament linters, `test_core` and both Gazebo
  launch tests. Concurrency cancels superseded runs; every job has a timeout.

## In progress

- Nothing at present.

## Not started

- **Nav2 end to end in CI.** The Nav2 bringup launches but there is no
  committed map and no test that sends a goal through the behaviour tree.
- **Accuracy of the global (`map`-frame) EKF** with AMCL. Configured and
  launched, not quantified.
- **Gazebo cross-check on more routes and speeds.** One route at one speed, by
  design for CI time; more would show whether any route separates the
  controllers in physics simulation.
- **arm64 / Jetson Orin container build.**
- **Plausibility check on fresh commands** (speed envelope, consistency with the
  plan) in the safety gate.

## Limitations

- **Simulation only.** No hardware bringup, no Jetson Orin, no physical
  emergency stop.
- **The safety layer is software.** If the gate process itself dies, the
  simulated base keeps executing its last command, as a motor controller
  without a firmware timeout would. A real base needs a firmware command
  timeout and a hardware e-stop that cuts motor power; neither is simulated.
- **The stop-time bounds are for this simulated base**, whose deceleration
  limit (4.0 m/s²) is carried over from the original model. A real drive would
  need them restated.
- **The headline comparison is the kinematic benchmark.** The Gazebo numbers
  are dominated by dead-reckoning drift and are a system check, not a
  controller ranking.
- **Gazebo runs are not bit-reproducible**, so the Gazebo section of the
  results file is refreshed by `scripts/run_gazebo_repeats.sh`, not
  regenerated in CI; CI runs the same scenario and checks the goal is reached
  and the metrics are produced, within sanity bounds.
- **Software rendering in CI.** The LiDAR and camera render on Mesa's CPU
  renderer; the real-time factor varies with the machine, which every node
  absorbs by running on the simulation clock.
- **Stanley on a differential drive** needs a virtual front axle distance,
  a tuning parameter of the adaptation; see `docs/CONTROLLERS.md`.
