// Pure Pursuit path tracker.
//
// Geometric law: pick a goal point one lookahead distance ahead on the path,
// then follow the unique circular arc from the robot's current pose through
// that point. For a point at (x_r, y_r) in the body frame at distance L_d, the
// arc curvature is
//
//     kappa = 2 * y_r / L_d^2
//
// and a differential drive realises it directly with omega = v * kappa.
//
// See docs/CONTROLLERS.md for the derivation and the tuning procedure.
#pragma once

#include "shuttle_control/path_tracker.hpp"

namespace shuttle
{

struct PurePursuitParams
{
  /// Lookahead is speed-scheduled: L_d = clamp(k_v * v + L_0, L_min, L_max).
  double lookahead_gain{1.0};  ///< k_v, seconds
  double lookahead_base{0.4};  ///< L_0, m
  double lookahead_min{0.5};   ///< m
  double lookahead_max{2.5};   ///< m

  /// Stop-and-rotate recovery for when the goal point falls behind the robot.
  ///
  /// The two thresholds give the recovery hysteresis. With a single threshold
  /// the controller chatters in and out of the recovery on alternate control
  /// steps, which pins the yaw rate at its limit and leaves the robot spinning
  /// on the spot without ever advancing along the path.
  double rotate_enter_angle{1.75};  ///< rad, ~100 deg
  double rotate_exit_angle{0.52};   ///< rad, ~30 deg
};

class PurePursuit : public PathTracker
{
public:
  PurePursuit() = default;
  explicit PurePursuit(const PurePursuitParams & params)
  : params_(params) {}

  const char * name() const override { return "pure_pursuit"; }

  PurePursuitParams & params() { return params_; }
  const PurePursuitParams & params() const { return params_; }

  Twist2D computeCommand(const VehicleState & state, double dt) override;
  void reset() override;

  /// True while the stop-and-rotate recovery is engaged.
  bool rotating() const { return rotating_; }

  /// Lookahead distance used on the most recent call, m.
  double lastLookahead() const { return last_lookahead_; }
  /// Goal point used on the most recent call, in the world frame.
  double lastGoalX() const { return last_goal_x_; }
  double lastGoalY() const { return last_goal_y_; }

private:
  PurePursuitParams params_;
  double last_lookahead_{0.0};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  bool rotating_{false};
};

}  // namespace shuttle
