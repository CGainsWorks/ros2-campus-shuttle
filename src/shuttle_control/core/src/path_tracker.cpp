#include "shuttle_control/path_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace shuttle
{

double SpeedLimiter::compute(double v_ref, double kappa, double dist_to_goal, double dt)
{
  double v = std::min(limits_.v_max, v_ref > 0.0 ? v_ref : limits_.v_max);

  // Lateral acceleration limit: v <= sqrt(a_lat_max / |kappa|).
  const double abs_k = std::fabs(kappa);
  if (abs_k > 1e-6) {
    v = std::min(v, std::sqrt(limits_.a_lat_max / abs_k));
  }

  // Decelerate into the goal.
  const double d = std::max(0.0, dist_to_goal);
  v = std::min(v, std::sqrt(2.0 * limits_.a_lon_max * d));

  // Keep creeping while there is still path left, otherwise the curvature limit
  // can stall the robot dead in a tight corner.
  if (d > 1e-3) {
    v = std::max(v, std::min(limits_.v_min_moving, std::sqrt(2.0 * limits_.a_lon_max * d)));
  }

  // Longitudinal acceleration rate limit.
  if (!initialised_) {
    v_prev_ = 0.0;
    initialised_ = true;
  }
  const double dv_max = limits_.a_lon_max * std::max(dt, 1e-6);
  v = clampValue(v, v_prev_ - dv_max, v_prev_ + dv_max);
  v = std::max(0.0, v);
  v_prev_ = v;
  return v;
}

void PathTracker::setPath(const Path & path)
{
  path_ = path;
  reset();
}

void PathTracker::reset()
{
  hint_ = 0;
  goal_reached_ = false;
  last_error_ = TrackingError{};
  speed_limiter_.setLimits(common_.speed);
  speed_limiter_.reset();
}

const TrackingError & PathTracker::updateError(const Pose2D & pose)
{
  // The search is always windowed around the previous projection, starting from
  // the head of the path. This keeps the projection on the branch the robot is
  // actually driving where the path crosses itself (the figure-eight scenario),
  // and it stops a closed loop from matching its own end point on the first
  // call and reporting the goal as reached before moving. Both the benchmark
  // and Nav2 hand over plans that begin at the robot, so starting the window at
  // index 0 is safe.
  last_error_ = path_.project(pose, hint_, common_.search_window);
  if (last_error_.valid) {
    hint_ = last_error_.index;
    // Arc length alone is not enough: require the robot to actually be near the
    // final point, so a loop cannot satisfy the test from the wrong lap.
    const double dx = pose.x - path_.back().x;
    const double dy = pose.y - path_.back().y;
    const bool near_end = std::hypot(dx, dy) <= 3.0 * common_.goal_tolerance;
    if (path_.remaining(last_error_.s) <= common_.goal_tolerance && near_end) {
      goal_reached_ = true;
    }
  }
  return last_error_;
}

}  // namespace shuttle
