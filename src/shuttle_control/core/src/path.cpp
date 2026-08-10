#include "shuttle_control/path.hpp"

#include <algorithm>
#include <cmath>

namespace shuttle
{
namespace
{

/// Signed Menger curvature of three consecutive points.
double mengerCurvature(
  double x0, double y0, double x1, double y1, double x2, double y2)
{
  const double ax = x1 - x0;
  const double ay = y1 - y0;
  const double bx = x2 - x1;
  const double by = y2 - y1;
  const double cx = x2 - x0;
  const double cy = y2 - y0;

  const double la = std::hypot(ax, ay);
  const double lb = std::hypot(bx, by);
  const double lc = std::hypot(cx, cy);
  const double denom = la * lb * lc;
  if (denom < 1e-12) {
    return 0.0;
  }
  const double cross = ax * by - ay * bx;
  return 2.0 * cross / denom;
}

}  // namespace

Path Path::fromPolyline(
  const std::vector<double> & xs,
  const std::vector<double> & ys,
  double nominal_speed)
{
  Path path;
  const std::size_t n = std::min(xs.size(), ys.size());
  if (n == 0) {
    return path;
  }

  std::vector<PathPoint> pts;
  pts.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (!pts.empty()) {
      const double d = std::hypot(xs[i] - pts.back().x, ys[i] - pts.back().y);
      if (d < 1e-6) {
        continue;  // duplicate sample: tangent would be undefined
      }
    }
    PathPoint p;
    p.x = xs[i];
    p.y = ys[i];
    p.v_ref = nominal_speed;
    pts.push_back(p);
  }

  if (pts.size() == 1) {
    path.points_ = pts;
    return path;
  }

  // Arc length and tangent heading.
  pts[0].s = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    pts[i].s = pts[i - 1].s + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    pts[i].yaw = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
  }
  pts.back().yaw = pts[pts.size() - 2].yaw;

  // Interior headings averaged with the incoming segment, so the tangent at a
  // corner is the bisector rather than the outgoing segment direction.
  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    const double in_yaw = std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
    const double out_yaw = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
    pts[i].yaw = wrapToPi(in_yaw + 0.5 * angleDiff(out_yaw, in_yaw));
  }

  // Signed curvature.
  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    pts[i].kappa = mengerCurvature(
      pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y);
  }
  if (pts.size() >= 3) {
    pts.front().kappa = pts[1].kappa;
    pts.back().kappa = pts[pts.size() - 2].kappa;
  }

  path.points_ = pts;
  return path;
}

PathPoint Path::interpolate(double s) const
{
  if (points_.empty()) {
    return PathPoint{};
  }
  if (points_.size() == 1 || s <= 0.0) {
    return points_.front();
  }
  if (s >= length()) {
    return points_.back();
  }

  // Binary search for the segment containing s.
  std::size_t lo = 0;
  std::size_t hi = points_.size() - 1;
  while (hi - lo > 1) {
    const std::size_t mid = (lo + hi) / 2;
    if (points_[mid].s <= s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  const PathPoint & a = points_[lo];
  const PathPoint & b = points_[hi];
  const double ds = b.s - a.s;
  const double t = ds > 1e-12 ? (s - a.s) / ds : 0.0;

  PathPoint p;
  p.x = a.x + t * (b.x - a.x);
  p.y = a.y + t * (b.y - a.y);
  p.yaw = wrapToPi(a.yaw + t * angleDiff(b.yaw, a.yaw));
  p.s = s;
  p.kappa = a.kappa + t * (b.kappa - a.kappa);
  p.v_ref = a.v_ref + t * (b.v_ref - a.v_ref);
  return p;
}

TrackingError Path::project(const Pose2D & pose, std::size_t hint, std::size_t window) const
{
  TrackingError err;
  if (points_.size() < 2) {
    if (points_.size() == 1) {
      err.valid = true;
      err.x = points_[0].x;
      err.y = points_[0].y;
      err.yaw = points_[0].yaw;
      err.cross_track = 0.0;
      err.heading = angleDiff(points_[0].yaw, pose.yaw);
    }
    return err;
  }

  std::size_t begin = 0;
  std::size_t end = points_.size() - 1;  // number of segments
  if (window > 0) {
    begin = hint > window ? hint - window : 0;
    end = std::min(points_.size() - 1, hint + window);
  }

  double best_d2 = -1.0;
  std::size_t best_i = begin;
  double best_t = 0.0;

  for (std::size_t i = begin; i < end; ++i) {
    const double ax = points_[i].x;
    const double ay = points_[i].y;
    const double bx = points_[i + 1].x;
    const double by = points_[i + 1].y;
    const double ex = bx - ax;
    const double ey = by - ay;
    const double seg2 = ex * ex + ey * ey;
    double t = 0.0;
    if (seg2 > 1e-12) {
      t = ((pose.x - ax) * ex + (pose.y - ay) * ey) / seg2;
      t = clampValue(t, 0.0, 1.0);
    }
    const double px = ax + t * ex;
    const double py = ay + t * ey;
    const double d2 = (pose.x - px) * (pose.x - px) + (pose.y - py) * (pose.y - py);
    if (best_d2 < 0.0 || d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
      best_t = t;
    }
  }

  const PathPoint & a = points_[best_i];
  const PathPoint & b = points_[best_i + 1];
  const double s = a.s + best_t * (b.s - a.s);
  const PathPoint p = interpolate(s);

  const double dx = pose.x - p.x;
  const double dy = pose.y - p.y;

  err.valid = true;
  err.index = best_i;
  err.s = s;
  err.x = p.x;
  err.y = p.y;
  err.yaw = p.yaw;
  err.kappa = p.kappa;
  // Left-positive lateral offset: project (robot - path) onto the path's left normal.
  err.cross_track = -std::sin(p.yaw) * dx + std::cos(p.yaw) * dy;
  err.heading = angleDiff(p.yaw, pose.yaw);
  return err;
}

TrackingError Path::project(const Pose2D & pose) const
{
  return project(pose, 0, 0);
}

}  // namespace shuttle
