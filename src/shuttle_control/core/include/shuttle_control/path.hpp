// Reference path: arc-length parameterisation, curvature, and projection.
#pragma once

#include <cstddef>
#include <vector>

#include "shuttle_control/types.hpp"

namespace shuttle
{

class Path
{
public:
  Path() = default;

  /// Build a path from a polyline. Tangent headings, cumulative arc length and
  /// signed curvature are derived from the geometry; `v_ref` is filled with
  /// `nominal_speed` for every point.
  ///
  /// Consecutive duplicate points (closer than 1e-6 m) are dropped, because they
  /// make the tangent heading undefined.
  static Path fromPolyline(
    const std::vector<double> & xs,
    const std::vector<double> & ys,
    double nominal_speed);

  bool empty() const { return points_.empty(); }
  std::size_t size() const { return points_.size(); }
  const std::vector<PathPoint> & points() const { return points_; }
  const PathPoint & at(std::size_t i) const { return points_[i]; }
  const PathPoint & front() const { return points_.front(); }
  const PathPoint & back() const { return points_.back(); }

  /// Total arc length, m.
  double length() const { return points_.empty() ? 0.0 : points_.back().s; }

  /// Interpolate the path at arc length `s` (clamped to the path extent).
  PathPoint interpolate(double s) const;

  /// Project `pose` onto the path.
  ///
  /// The search is global by default. Passing a `hint` index restricts the
  /// search to a window around the previous projection, which keeps the
  /// behaviour well defined on self-intersecting paths such as a figure eight.
  TrackingError project(const Pose2D & pose, std::size_t hint, std::size_t window) const;
  TrackingError project(const Pose2D & pose) const;

  /// Remaining arc length from `s` to the end of the path.
  double remaining(double s) const { return length() - s; }

private:
  std::vector<PathPoint> points_;
};

}  // namespace shuttle
