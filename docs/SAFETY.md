# Safety case

This repository is a **simulation**. Nothing here has been run on a vehicle
carrying people, and the analysis below is what a hardware bringup would have to
satisfy, not a record of one that happened. It is written in the present tense
because that is how a safety case is written, but every claim about hardware is
a requirement, not a result.

## Scope

A low-speed (≤ 1.5 m/s) differential-drive shuttle on pedestrian campus paths,
carrying no passengers, always within line of sight of a trained operator.

## What can go wrong

| Hazard | Cause | Mitigation | Status |
| --- | --- | --- | --- |
| Collision with a pedestrian | Perception misses a low or thin obstacle; 2D LiDAR sees one horizontal slice only | Speed limited to 1.5 m/s; Nav2 inflation radius 0.65 m against a 0.55 m robot radius; operator supervision | 2D-only sensing is a real limitation, see below |
| Collision with a static obstacle | Localisation drift; costmap not updated | Global costmap from a prior map plus live scan; local rolling costmap | Implemented in simulation |
| Runaway on a control fault | Controller commands a velocity the operator cannot override | Hardware emergency stop cutting motor power, independent of software | **Hardware only, not simulated** |
| Loss of localisation | Scan matching fails in a feature-poor area | EKF covariance monitored; `ekf_evaluator` reports NEES | Monitoring implemented but **not yet verified end to end**; the stop-on-divergence behaviour is not implemented at all |
| Stale command | A node dies while the last `cmd_vel` is still being executed | The base must stop on command timeout | **Not implemented**: `gazebo_ros_diff_drive` has no command timeout, so a real base needs one in firmware |
| Tracking failure on tight corners | Controller limit-cycles or cuts a corner | Benchmarked over seven scenarios including corners tighter than the tuning set; stalls and divergence are recorded as explicit outcomes | Implemented and tested |

## The emergency stop

The brief lists a physical emergency stop. It must cut **motor power in
hardware**, not request a stop in software. A software stop shares the failure
modes of the thing it is supposed to protect against: if the controller node has
hung, a stop command routed through the same stack does not arrive.

This is not simulated and cannot be. It is the first thing that would have to be
built and tested for a hardware bringup.

## Known limitations that bear on safety

- **The LiDAR is 2D.** It sees one horizontal plane at 0.32 m. A kerb, a
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
- **No fault detection on the controller output.** Nothing checks that the
  commanded velocity is sane before it reaches the wheels.

## What the simulation does establish

- The controllers do not diverge or stall on any of the 84 benchmark runs,
  including scenarios they were not tuned on.
- Speed is bounded by an explicit lateral-acceleration limit and a
  deceleration-to-goal limit, shared by both controllers.
- Wheel saturation is modelled, so commanded velocity pairs a differential drive
  cannot physically execute are scaled rather than silently clipped.
- Tracking error is measured against ground truth, so a controller that tracks a
  drifting estimate cannot score well.
