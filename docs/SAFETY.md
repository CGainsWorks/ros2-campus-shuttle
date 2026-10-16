# Safety case

This repository is a **simulation**. Nothing here has been run on a vehicle
carrying people and the analysis below is what a hardware bringup would have to
satisfy, not a record of one that happened. It is written in the present tense
because that is how a safety case is written but every claim about hardware is
a requirement, not a result.

## Scope

A low-speed (≤ 1.5 m/s) differential-drive shuttle on pedestrian campus paths,
carrying no passengers, always within line of sight of a trained operator.

## What can go wrong

| Hazard | Cause | Mitigation | Status |
| --- | --- | --- | --- |
| Collision with a pedestrian | Perception misses a low or thin obstacle; 2D LiDAR sees one horizontal slice only | Speed limited to 1.5 m/s; Nav2 inflation radius 0.65 m against a 0.55 m robot radius; operator supervision | 2D-only sensing is a real limitation, see below |
| Collision with a static obstacle | Localisation drift; costmap not updated | Global costmap from a prior map plus live scan; local rolling costmap | Implemented in simulation |
| Runaway on a control fault | Controller commands a velocity the operator cannot override | Software e-stop in the safety gate (latched, drops all commands); hardware emergency stop cutting motor power, independent of software | Software e-stop **implemented and tested in simulation**; hardware stop **not simulated** |
| Loss of localisation | Scan matching fails in a feature-poor area | `ekf_evaluator` reports drift and NEES against ground truth | Measured in Gazebo: 0.204 to 0.442 m RMS drift over a 22.7 m dead-reckoned run. Covariance is unbounded in this configuration, so it cannot be used as a divergence trigger; the stop-on-divergence behaviour is **not** implemented |
| Stale command | A node dies while the last `cmd_vel` is still being executed | Command-timeout watchdog in the safety gate; a firmware timeout in the base | Watchdog **implemented and tested in simulation**. The simulated differential drive, like a motor controller without a firmware timeout, executes its last command for ever, so if the gate itself dies nothing stops it; the firmware timeout is **not simulated** |
| Tracking failure on tight corners | Controller limit-cycles or cuts a corner | Benchmarked over seven scenarios including corners tighter than the tuning set; stalls and divergence are recorded as explicit outcomes | Implemented and tested |

## The safety layer in simulation

Every command source publishes on `/cmd_vel_in`. `safety_gate_node`
(`shuttle_control`) is the only publisher of `/cmd_vel`, which is what the
simulated base executes:

```
path tracker / Nav2 velocity smoother / teleop
        |  /cmd_vel_in
        v
  safety_gate_node  <-- /e_stop (std_msgs/Bool), /set_e_stop (std_srvs/SetBool)
        |  /cmd_vel           --> /e_stop/active (std_msgs/Bool, latched)
        v
  ros_gz_bridge --> Gazebo differential drive
```

- **Command timeout.** With no command for 0.5 s (simulated time), the gate
  publishes zero and keeps the base held at zero until a fresh command arrives.
  This is the 0.5 s command timeout, a launch argument (`cmd_timeout`).
- **E-stop.** Engaging it, by topic or service, publishes zero immediately,
  drops every command while it is held and publishes zero at 50 Hz so a source
  that bypassed the gate could not hold the base in motion either. Releasing it
  does not resume anything: motion restarts only on a command sent after the
  release.

The logic is `CommandGate` in the ROS-free core, unit tested in `test_core` on
synthetic time (timeout boundary, clock running backwards after a simulator
reset, commands dropped during a stop, no resumption on release).

### Stated stop times and how they are proved

Requirement, from 1.0 m/s on level ground: the base is at rest (ground-truth
speed below 0.05 m/s, from the physics engine, not from odometry) within 0.5 s
of the e-stop and within 1.0 s of the last command when commands stop.

`src/shuttle_bringup/test/test_safety_sim.py` (launch_testing) starts Gazebo
headless with the gate in front of the base, drives at 1.0 m/s and checks both
on every CI run. Measured (simulated time, Gazebo Harmonic): from 1.00 m/s to
rest in 0.26 s after the e-stop, with full-speed commands still arriving at
20 Hz; and at rest 0.77 s after the last command for the timeout, of which 0.5 s
is the timeout itself. The test also checks the base was still moving partway
into the timeout window, so it is the watchdog that stopped it and that it
stays stopped after the e-stop is released.

The braking itself is set by the simulated drive's deceleration limit of
4.0 m/s², carried over from the Gazebo Classic model, so about 0.25 s from
1.0 m/s. A real base would brake at whatever its drives allow and the bound
would have to be restated for it.

## The emergency stop in hardware

The brief lists a physical emergency stop. It must cut **motor power in
hardware**, not request a stop in software. A software stop shares the failure
modes of the thing it is supposed to protect against: if the controller node has
hung, a stop command routed through the same stack does not arrive.

The software e-stop above is the stop an operator console or a supervisory node
would use. It is not a substitute for the hardware one, which is not simulated
and cannot be. That is the first thing that would have to be built and tested
for a hardware bringup, together with a command timeout in the motor controller
firmware, so that a dead gate also stops the base.

## Known limitations that bear on safety

- **The LiDAR is 2D.** It sees one horizontal plane at 0.37 m. A kerb, a
  bollard cap, an overhang or a child's outstretched arm outside that plane is
  invisible. This is adequate for a supervised demonstration and is not adequate
  for unsupervised operation around people.
- **No dynamic obstacle prediction.** Pedestrians are treated as static
  obstacles refreshed each costmap update. The shuttle reacts, it does not
  anticipate.
- **The comparison runs disable AMCL** and use ground truth for the map frame,
  deliberately, so localisation error does not contaminate the controller
  measurement. That is right for the experiment and wrong for operation: the
  numbers in RESULTS.md are tracking error, not the total path error a real
  robot would show.
- **No plausibility check on the controller output.** The gate stops stale
  commands and honours the e-stop but nothing checks that a fresh command is
  sane (within the speed envelope, consistent with the plan) before it reaches
  the wheels.

## What the simulation does establish

- The controllers do not diverge or stall on any of the 84 benchmark runs,
  including scenarios they were not tuned on.
- Speed is bounded by an explicit lateral-acceleration limit and a
  deceleration-to-goal limit, shared by both controllers.
- Wheel saturation is modelled, so commanded velocity pairs a differential drive
  cannot physically execute are scaled rather than silently clipped.
- Tracking error is measured against ground truth, so a controller that tracks a
  drifting estimate cannot score well.
- The software e-stop and the command timeout bring the simulated base to rest
  within their stated times, with commands still arriving in the e-stop case.
