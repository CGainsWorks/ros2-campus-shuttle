// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Common interface and shared longitudinal control for the path trackers.
#pragma once

#include <string>

#include "shuttle_control/path.hpp"
#include "shuttle_control/types.hpp"

namespace shuttle
{

/// Longitudinal (speed) control, shared verbatim by both lateral controllers.
///
/// This is deliberately factored out. Pure Pursuit and Stanley differ only in
/// how they choose yaw rate; if they also disagreed about speed, any difference
/// in tracking error would be confounded by a difference in how fast the robot
/// happened to be taking the corner. Keeping the speed law identical is what
/// makes the comparison like-for-like.
struct SpeedLimits
{
  double v_max{1.5};          ///< m/s
  double a_lon_max{0.8};      ///< m/s^2, used for both accel and decel-to-goal
  double a_lat_max{0.8};      ///< m/s^2, curvature-based speed limit
  double v_min_moving{0.15};  ///< m/s, floor while still away from the goal
};

class SpeedLimiter
{
public:
  explicit SpeedLimiter(const SpeedLimits & limits = SpeedLimits())
  : limits_(limits) {}

  void setLimits(const SpeedLimits & limits) {limits_ = limits;}
  const SpeedLimits & limits() const {return limits_;}
  void reset() {v_prev_ = 0.0; initialised_ = false;}

  /// Compute the commanded speed.
  /// @param v_ref        reference speed carried by the path
  /// @param kappa        curvature the robot is about to drive through
  /// @param dist_to_goal remaining arc length, m
  /// @param dt           control period, s
  double compute(double v_ref, double kappa, double dist_to_goal, double dt);

private:
  SpeedLimits limits_;
  double v_prev_{0.0};
  bool initialised_{false};
};

/// Parameters that apply to every tracker.
struct TrackerCommon
{
  SpeedLimits speed;
  double goal_tolerance{0.25};       ///< m, remaining arc length counted as "arrived"
  double max_omega{1.5};             ///< rad/s
  double control_period{0.05};       ///< s, nominal; the actual dt is passed in
  std::size_t search_window{80};     ///< projection search window, samples
};

class PathTracker
{
public:
  virtual ~PathTracker() = default;

  virtual const char * name() const = 0;

  virtual void setPath(const Path & path);
  virtual void reset();

  /// Compute a velocity command for the current state.
  virtual Twist2D computeCommand(const VehicleState & state, double dt) = 0;

  const TrackingError & lastError() const {return last_error_;}
  bool goalReached() const {return goal_reached_;}
  const Path & path() const {return path_;}

  TrackerCommon & common() {return common_;}
  const TrackerCommon & common() const {return common_;}

protected:
  /// Project the pose, update `last_error_` and the goal flag.
  const TrackingError & updateError(const Pose2D & pose);

  Path path_;
  TrackerCommon common_;
  SpeedLimiter speed_limiter_;
  TrackingError last_error_;
  std::size_t hint_{0};
  bool goal_reached_{false};
};

}  // namespace shuttle
