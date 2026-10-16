# The two path trackers

Both controllers live in `src/shuttle_control/core/`, which has no ROS
dependency. The same objects are linked into the ROS 2 node, the Nav2 plugins
and the offline benchmark, so a result measured in one setting holds in the
others.

## Conventions

Cross-track error `e` is **positive when the robot is to the left of the path
tangent**. Heading error is `psi = wrapToPi(path_yaw - robot_yaw)`, so positive
means the robot must turn left to align.

Both controllers report their error **at `base_link`**. This matters: Stanley
internally regulates the error at a virtual front axle, which is a different and
generally smaller quantity. Reporting at a common reference point is what makes
the two columns of the results table comparable.

## Pure Pursuit

Pick a goal point one lookahead distance along the path and follow the unique
circular arc from the current pose through it. For a goal at `(x_r, y_r)` in the
body frame at distance `L_d`:

```
kappa = 2 * y_r / L_d^2
omega = v * kappa
```

The lookahead is speed-scheduled:

```
L_d = clamp(k_v * v + L_0, L_min, L_max)
```

Two implementation choices are worth stating.

**The goal point is found by arc length, not by circle intersection.** The
textbook formulation intersects a circle of radius `L_d` with the path, which is
ambiguous when the path curves back within the lookahead: on the figure-eight
scenario there can be several intersections. Advancing `L_d` along the arc from
the current projection is unambiguous and behaves identically on paths where the
two agree.

**Stop-and-rotate recovery has hysteresis.** When the goal point falls well
behind the robot the pursuit arc degenerates and turning on the spot is the
right response. Gating that on the sign of `x_r` alone makes the controller flip
in and out of recovery on alternate control steps, pinning the yaw rate at its
limit while the speed is repeatedly reset to zero, so the robot spins and never
advances. Entry at ~100° and exit at ~30° removes the chatter.

On a constant-radius arc, Pure Pursuit's commanded curvature equals `1/R`
exactly. That is why it tracks the `circle` scenario to a fraction of a
millimetre: the geometry is not an approximation there, it is exact.

## Stanley, adapted to a differential drive

The Stanley law is defined for a **front-wheel-steered** vehicle. It regulates
the cross-track error at the front axle and outputs a steering angle:

```
delta = psi + atan2(k_e * e_fa, k_soft + v)
```

A differential drive has neither a front axle nor a steering angle, so two
things must be supplied:

1. **A virtual front axle** a distance `L_fa` ahead of `base_link`. The error
   `e_fa` is measured there rather than at the robot origin.
2. **A steering-to-yaw-rate conversion**, using the bicycle relation
   `omega = v * tan(delta) / L_fa`.

`L_fa` therefore plays the role of a wheelbase. **It is a tuning parameter of
this adaptation, not a property of the robot** and it materially changes the
behaviour: it sets both how far ahead the error is sensed and how aggressively a
given steering angle is realised as yaw rate. This is the single largest caveat
on the comparison and anyone reading the result should know that a different
`L_fa` moves Stanley's numbers.

The sign of the feedback term is negated relative to the textbook form, because
this codebase defines `e` as positive-to-the-left while the original defines it
the other way round. `testSteeringSigns()` pins this down: a flipped convention
is the classic way to get a path tracker quietly wrong and it is the kind of
error that still looks plausible on a straight line.

`k_damp` exposes the later yaw-damping extension. It defaults to zero so the
comparison is against the published law rather than a local variant.

## What is shared and why

The **longitudinal controller is shared code** (`SpeedLimiter`). Both
controllers pick speed as

```
v = min(v_max, v_ref, sqrt(a_lat_max / |kappa|), sqrt(2 * a_lon_max * d_remaining))
```

rate-limited by `a_lon_max`.

This is the most important fairness decision in the repository. If the two
controllers also disagreed about speed, any difference in tracking error could
be explained by one of them simply taking the corner slower. Sharing the speed
law means the only difference between the two result columns is the lateral
control law.

## Tuning

`controller_bench --tune` grid-searches both controllers against

```
cost = RMS_cross_track / 0.05 m + 0.5 * control_effort / 5 rad/s^2
```

with a large penalty for any run that diverges, stalls or fails to arrive.

The effort term is not decoration. Minimising tracking error alone drives Pure
Pursuit to the shortest lookahead on the grid and Stanley to the highest
cross-track gain and both of those sit on the edge of oscillation. The two
reference scales say what "clearly poor" looks like for each quantity and the
weight says accuracy counts twice as much as smoothness.

Two things about the setup are deliberate:

- **The lookahead lower bound is tuned, not fixed.** At the benchmark speeds the
  clamp is what actually binds, so holding `lookahead_min` constant would leave
  low-speed behaviour outside the search entirely.
- **The tuning set is a subset of the scenarios.** Gains are fitted on
  `straight_offset`, `circle`, `corner` and `campus_loop`; `heading_offset`,
  `figure_eight` and `sharp_turns` are held out, so the reported table includes
  routes neither controller was tuned on. `sharp_turns` uses 1.2 m corners,
  tighter than the 1.5 m corner in the tuning set, which is a real
  generalisation test.

The grid deliberately extends below any value one would pick by hand, so the
selected gains can be seen to be an interior optimum rather than whatever the
lower bound happened to be.

## Sensing latency

The benchmark models a 100 ms delay between measurement and actuation, which is
representative of a scan-match plus EKF plus ROS transport pipeline.

This is not a detail. With a latency-free plant the tuner drives the Pure
Pursuit lookahead down to 0.15 m, because in a perfect world a shorter lookahead
simply tracks better. Latency is what makes an unreasonably short lookahead
oscillate. Without it, the tuning step selects gains that would be unusable on a
real robot and the comparison would be between two controllers tuned for a
world that does not exist.

## Results

See [RESULTS.md](RESULTS.md), which is generated from the benchmark output.
