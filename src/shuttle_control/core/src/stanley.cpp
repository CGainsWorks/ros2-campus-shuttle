// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "shuttle_control/stanley.hpp"

#include <algorithm>
#include <cmath>

namespace shuttle
{

Twist2D Stanley::computeCommand(const VehicleState & state, double dt)
{
  Twist2D cmd;
  if (path_.size() < 2) {
    return cmd;
  }

  // The *reported* error is always measured at base_link, so that the metrics
  // are directly comparable with Pure Pursuit. The control law separately uses
  // the error at the virtual front axle below.
  const TrackingError & err = updateError(state.pose);
  if (!err.valid) {
    return cmd;
  }
  if (goal_reached_) {
    speed_limiter_.reset();
    return cmd;
  }

  const double v = speed_limiter_.compute(
    path_.interpolate(err.s).v_ref,
    err.kappa,
    path_.remaining(err.s),
    dt);

  // Virtual front axle pose.
  Pose2D front;
  front.x = state.pose.x + params_.front_axle_distance * std::cos(state.pose.yaw);
  front.y = state.pose.y + params_.front_axle_distance * std::sin(state.pose.yaw);
  front.yaw = state.pose.yaw;

  const TrackingError fa = path_.project(front, hint_, common_.search_window);
  if (!fa.valid) {
    return cmd;
  }
  last_fa_error_ = fa.cross_track;

  // delta = psi + atan2(-k_e * e_left, k_soft + v)
  //
  // Negated because e_left > 0 means the robot sits to the left of the path and
  // must steer right (negative delta) to rejoin it.
  const double heading_term = fa.heading;
  const double speed_for_gain = params_.k_soft + std::fabs(v);
  const double cross_term = std::atan2(-params_.k_e * fa.cross_track, speed_for_gain);

  double delta = heading_term + cross_term;
  if (params_.k_damp > 0.0) {
    // Damp against the yaw rate the path curvature already accounts for.
    const double omega_ff = v * err.kappa;
    delta -= params_.k_damp * (state.omega - omega_ff);
  }
  delta = clampValue(delta, -params_.max_steer, params_.max_steer);
  last_steer_ = delta;

  // Bicycle relation, with the virtual front-axle distance as the wheelbase.
  const double omega = v * std::tan(delta) / std::max(params_.front_axle_distance, 1e-3);

  cmd.v = v;
  cmd.omega = clampValue(omega, -common_.max_omega, common_.max_omega);
  return cmd;
}

}  // namespace shuttle
