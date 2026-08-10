// Basic 2D types shared by the path trackers, the plant model and the metrics.
//
// This header (and everything else under core/) deliberately depends on nothing
// but the C++ standard library. The same translation units are compiled into the
// ROS 2 node, into the Nav2 plugins and into the offline benchmark, so the code
// under comparison is identical in all three settings.
#pragma once

#include <cmath>
#include <cstddef>

namespace shuttle
{

constexpr double kPi = 3.14159265358979323846;

/// Wrap an angle to (-pi, pi].
inline double wrapToPi(double angle)
{
  double a = std::fmod(angle + kPi, 2.0 * kPi);
  if (a <= 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

/// Shortest signed angular difference from `from` to `to`.
inline double angleDiff(double to, double from)
{
  return wrapToPi(to - from);
}

inline double clampValue(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int signOf(double v)
{
  return (v > 0.0) - (v < 0.0);
}

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Body-frame velocity command for a differential-drive base.
struct Twist2D
{
  double v{0.0};      ///< linear velocity, m/s
  double omega{0.0};  ///< yaw rate, rad/s
};

/// Everything a tracker is allowed to see about the robot.
struct VehicleState
{
  Pose2D pose;
  double v{0.0};      ///< measured linear velocity, m/s
  double omega{0.0};  ///< measured yaw rate, rad/s
  double t{0.0};      ///< seconds since the run started
};

/// One sample of the reference path.
struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};    ///< tangent heading, rad
  double s{0.0};      ///< arc length from the start of the path, m
  double kappa{0.0};  ///< signed curvature, 1/m (positive = left turn)
  double v_ref{0.0};  ///< reference speed for this point, m/s
};

/// Result of projecting a pose onto the reference path.
///
/// `cross_track` is positive when the robot is to the **left** of the path
/// tangent. The same convention and the same reference point (base_link) are
/// used for both controllers, so the reported numbers are directly comparable
/// even though Stanley internally regulates the error at a virtual front axle.
struct TrackingError
{
  double cross_track{0.0};
  double heading{0.0};  ///< wrapToPi(path_yaw - robot_yaw)
  double s{0.0};        ///< arc length of the projection
  double x{0.0};        ///< projected point on the path
  double y{0.0};
  double yaw{0.0};       ///< path tangent at the projection
  double kappa{0.0};     ///< path curvature at the projection
  std::size_t index{0};  ///< index of the segment start
  bool valid{false};
};

}  // namespace shuttle
