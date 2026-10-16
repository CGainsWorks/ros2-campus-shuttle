// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "shuttle_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace shuttle
{

void PurePursuit::reset()
{
  PathTracker::reset();
  rotating_ = false;
}

Twist2D PurePursuit::computeCommand(const VehicleState & state, double dt)
{
  Twist2D cmd;
  if (path_.size() < 2) {
    return cmd;
  }

  const TrackingError & err = updateError(state.pose);
  if (!err.valid) {
    return cmd;
  }
  if (goal_reached_) {
    speed_limiter_.reset();
    return cmd;
  }

  // Speed-scheduled lookahead.
  const double speed_for_lookahead = std::max(0.0, state.v);
  const double lookahead = clampValue(
    params_.lookahead_gain * speed_for_lookahead + params_.lookahead_base,
    params_.lookahead_min,
    params_.lookahead_max);
  last_lookahead_ = lookahead;

  // Goal point one lookahead of arc length ahead of the projection. Advancing
  // by arc length rather than by circle intersection keeps the goal point
  // unambiguous when the path curves back on itself.
  const PathPoint goal = path_.interpolate(err.s + lookahead);
  last_goal_x_ = goal.x;
  last_goal_y_ = goal.y;

  // Goal point in the body frame.
  const double dx = goal.x - state.pose.x;
  const double dy = goal.y - state.pose.y;
  const double cy = std::cos(state.pose.yaw);
  const double sy = std::sin(state.pose.yaw);
  const double x_r = cy * dx + sy * dy;
  const double y_r = -sy * dx + cy * dy;

  const double ld2 = x_r * x_r + y_r * y_r;
  double kappa = 0.0;
  if (ld2 > 1e-9) {
    kappa = 2.0 * y_r / ld2;
  }

  // Longitudinal command uses the worse of the curvature at the projection and
  // the curvature of the arc being followed, so the speed law sees the corner.
  const double kappa_for_speed = std::max(std::fabs(err.kappa), std::fabs(kappa));
  const double v = speed_limiter_.compute(
    err.valid ? path_.interpolate(err.s).v_ref : common_.speed.v_max,
    kappa_for_speed,
    path_.remaining(err.s),
    dt);

  // Stop-and-rotate recovery, with hysteresis on the bearing to the goal point.
  //
  // If the goal point ends up well behind the robot (a large disturbance, or a
  // plan that starts pointing the other way) the pursuit arc degenerates, and
  // turning on the spot is the sane response. The hysteresis matters: gating
  // this on the sign of x_r alone makes the controller flip in and out of
  // recovery on alternate control steps at a tight corner, which holds the yaw
  // rate at its limit while the speed is repeatedly reset to zero, and the
  // robot spins on the spot indefinitely.
  const double bearing = std::atan2(y_r, x_r);
  if (!rotating_ && std::fabs(bearing) > params_.rotate_enter_angle) {
    rotating_ = true;
  } else if (rotating_ && std::fabs(bearing) < params_.rotate_exit_angle) {
    rotating_ = false;
  }

  if (rotating_) {
    cmd.v = 0.0;
    cmd.omega = clampValue(2.0 * bearing, -common_.max_omega, common_.max_omega);
    speed_limiter_.reset();
    return cmd;
  }

  cmd.v = v;
  cmd.omega = clampValue(v * kappa, -common_.max_omega, common_.max_omega);
  return cmd;
}

}  // namespace shuttle
