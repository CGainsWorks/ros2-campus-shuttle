// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "shuttle_control/scenarios.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace shuttle
{
namespace
{

constexpr double kSampleStep = 0.05;  // m between path samples

void appendLine(
  std::vector<double> & xs, std::vector<double> & ys,
  double x0, double y0, double x1, double y1)
{
  const double len = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(len / kSampleStep));
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    xs.push_back(x0 + t * (x1 - x0));
    ys.push_back(y0 + t * (y1 - y0));
  }
}

/// Append a circular arc centred at (cx, cy) from angle a0 to a1.
void appendArc(
  std::vector<double> & xs, std::vector<double> & ys,
  double cx, double cy, double radius, double a0, double a1)
{
  const double sweep = a1 - a0;
  const double len = std::fabs(sweep) * radius;
  const int n = std::max(1, static_cast<int>(len / kSampleStep));
  for (int i = 1; i <= n; ++i) {
    const double a = a0 + sweep * static_cast<double>(i) / n;
    xs.push_back(cx + radius * std::cos(a));
    ys.push_back(cy + radius * std::sin(a));
  }
}

/// Build a path through `waypoints`, replacing each interior corner with a
/// circular fillet of the given radius.
///
/// Chaining appendLine/appendArc by hand and tracking the pen position in
/// separate variables is error prone: an earlier version of the staircase
/// scenario advanced its bookkeeping past a corner and emitted a path that
/// doubled back on itself by more than a metre, which no controller can track.
/// Deriving the tangent points from the geometry removes that class of mistake,
/// and testPathContinuity() checks the result.
Path makeFilleted(
  const std::vector<std::pair<double, double>> & waypoints, double radius, double v)
{
  std::vector<double> xs;
  std::vector<double> ys;
  if (waypoints.size() < 2) {
    return Path();
  }

  xs.push_back(waypoints.front().first);
  ys.push_back(waypoints.front().second);
  double cx = waypoints.front().first;
  double cy = waypoints.front().second;

  for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
    const double px = waypoints[i - 1].first;
    const double py = waypoints[i - 1].second;
    const double x = waypoints[i].first;
    const double y = waypoints[i].second;
    const double nx = waypoints[i + 1].first;
    const double ny = waypoints[i + 1].second;

    const double l1 = std::hypot(x - px, y - py);
    const double l2 = std::hypot(nx - x, ny - y);
    if (l1 < 1e-9 || l2 < 1e-9) {
      continue;
    }
    const double v1x = (x - px) / l1;
    const double v1y = (y - py) / l1;
    const double v2x = (nx - x) / l2;
    const double v2y = (ny - y) / l2;

    const double cross = v1x * v2y - v1y * v2x;
    const double dot = v1x * v2x + v1y * v2y;
    const double delta = std::atan2(cross, dot);  // signed deflection
    if (std::fabs(delta) < 1e-9) {
      continue;  // collinear, nothing to round
    }

    // Tangent length, capped so neighbouring fillets cannot overlap.
    const double half_turn = std::fabs(delta) / 2.0;
    double t_len = radius * std::tan(half_turn);
    t_len = std::min(t_len, 0.5 * std::min(l1, l2));
    const double r_eff = t_len / std::tan(half_turn);

    const double tix = x - v1x * t_len;
    const double tiy = y - v1y * t_len;
    const double tox = x + v2x * t_len;
    const double toy = y + v2y * t_len;

    appendLine(xs, ys, cx, cy, tix, tiy);

    // Arc centre lies on the inward normal of the incoming segment.
    const int turn = delta > 0.0 ? 1 : -1;
    const double ccx = tix + turn * (-v1y) * r_eff;
    const double ccy = tiy + turn * (v1x) * r_eff;

    const double a0 = std::atan2(tiy - ccy, tix - ccx);
    const double a1 = std::atan2(toy - ccy, tox - ccx);
    double sweep = a1 - a0;
    // Take the sweep that matches the turn direction.
    while (turn > 0 && sweep < 0.0) {sweep += 2.0 * kPi;}
    while (turn < 0 && sweep > 0.0) {sweep -= 2.0 * kPi;}

    appendArc(xs, ys, ccx, ccy, r_eff, a0, a0 + sweep);
    cx = tox;
    cy = toy;
  }

  appendLine(xs, ys, cx, cy, waypoints.back().first, waypoints.back().second);
  return Path::fromPolyline(xs, ys, v);
}

Scenario straightOffset(double v)
{
  Scenario sc;
  sc.name = "straight_offset";
  sc.description = "30 m straight, robot starts 1.0 m laterally off the path";
  std::vector<double> xs{0.0};
  std::vector<double> ys{0.0};
  appendLine(xs, ys, 0.0, 0.0, 30.0, 0.0);
  sc.path = Path::fromPolyline(xs, ys, v);
  sc.start = Pose2D{0.0, 1.0, 0.0};
  sc.timeout = 180.0;
  return sc;
}

Scenario headingOffset(double v)
{
  Scenario sc;
  sc.name = "heading_offset";
  sc.description = "30 m straight, robot starts on the path with 30 deg heading error";
  std::vector<double> xs{0.0};
  std::vector<double> ys{0.0};
  appendLine(xs, ys, 0.0, 0.0, 30.0, 0.0);
  sc.path = Path::fromPolyline(xs, ys, v);
  sc.start = Pose2D{0.0, 0.0, 30.0 * kPi / 180.0};
  sc.timeout = 180.0;
  return sc;
}

Scenario circle(double v)
{
  Scenario sc;
  sc.name = "circle";
  sc.description = "single 6 m radius lap, constant curvature";
  const double r = 6.0;
  std::vector<double> xs{r};
  std::vector<double> ys{0.0};
  appendArc(xs, ys, 0.0, 0.0, r, 0.0, 2.0 * kPi);
  sc.path = Path::fromPolyline(xs, ys, v);
  // Start on the path, tangent heading (counter-clockwise => +y at angle 0).
  sc.start = Pose2D{r, 0.0, kPi / 2.0};
  sc.timeout = 240.0;
  return sc;
}

Scenario figureEight(double v)
{
  Scenario sc;
  sc.name = "figure_eight";
  sc.description = "lemniscate, curvature reverses sign twice, path self-intersects";
  const double a = 8.0;
  std::vector<double> xs;
  std::vector<double> ys;
  const int n = 2400;
  for (int i = 0; i <= n; ++i) {
    const double t = 2.0 * kPi * static_cast<double>(i) / n;
    xs.push_back(a * std::sin(t));
    ys.push_back(a * std::sin(t) * std::cos(t));
  }
  sc.path = Path::fromPolyline(xs, ys, v);
  // Tangent at t = 0 is (a, a) normalised => 45 degrees.
  sc.start = Pose2D{0.0, 0.0, kPi / 4.0};
  sc.timeout = 300.0;
  return sc;
}

Scenario campusLoop(double v)
{
  Scenario sc;
  sc.name = "campus_loop";
  sc.description = "40 x 25 m block with 3 m rounded corners, the realistic route";
  const double r = 3.0;
  const double w = 40.0;
  const double h = 25.0;
  std::vector<double> xs{r};
  std::vector<double> ys{0.0};
  appendLine(xs, ys, r, 0.0, w - r, 0.0);
  appendArc(xs, ys, w - r, r, r, -kPi / 2.0, 0.0);
  appendLine(xs, ys, w, r, w, h - r);
  appendArc(xs, ys, w - r, h - r, r, 0.0, kPi / 2.0);
  appendLine(xs, ys, w - r, h, r, h);
  appendArc(xs, ys, r, h - r, r, kPi / 2.0, kPi);
  appendLine(xs, ys, 0.0, h - r, 0.0, r);
  appendArc(xs, ys, r, r, r, kPi, 1.5 * kPi);
  sc.path = Path::fromPolyline(xs, ys, v);
  sc.start = Pose2D{r, 0.0, 0.0};
  sc.timeout = 360.0;
  return sc;
}

Scenario corner(double v)
{
  Scenario sc;
  sc.name = "corner";
  sc.description = "8 m straights joined by a 1.5 m radius left then right 90 deg turn";
  const double r = 1.5;
  std::vector<double> xs{0.0};
  std::vector<double> ys{0.0};
  appendLine(xs, ys, 0.0, 0.0, 8.0 - r, 0.0);
  // Left turn: centre above the path, sweeping -pi/2 -> 0.
  appendArc(xs, ys, 8.0 - r, r, r, -kPi / 2.0, 0.0);
  appendLine(xs, ys, 8.0, r, 8.0, 8.0 - r);
  // Right turn: centre to the east, sweeping pi -> pi/2.
  appendArc(xs, ys, 8.0 + r, 8.0 - r, r, kPi, kPi / 2.0);
  appendLine(xs, ys, 8.0 + r, 8.0, 16.0, 8.0);
  sc.path = Path::fromPolyline(xs, ys, v);
  sc.start = Pose2D{0.0, 0.0, 0.0};
  sc.timeout = 240.0;
  return sc;
}

Scenario sharpTurns(double v)
{
  Scenario sc;
  sc.name = "sharp_turns";
  sc.description = "serpentine of 1.2 m radius 90 deg corners, worst case for corner cutting";
  // Serpentine: east, north, west, north, ... Every interior vertex is a 90 deg
  // turn, alternating left and right, rounded to 1.2 m -- tighter than anything
  // in the tuning set.
  const double leg = 6.0;
  std::vector<std::pair<double, double>> wps;
  wps.emplace_back(0.0, 0.0);
  for (int i = 0; i < 4; ++i) {
    const double y0 = 2.0 * leg * i;
    wps.emplace_back(leg, y0);
    wps.emplace_back(leg, y0 + leg);
    wps.emplace_back(0.0, y0 + leg);
    wps.emplace_back(0.0, y0 + 2.0 * leg);
  }
  sc.path = makeFilleted(wps, 1.2, v);
  sc.start = Pose2D{0.0, 0.0, 0.0};
  sc.timeout = 420.0;
  return sc;
}

}  // namespace

std::vector<std::string> scenarioNames()
{
  return {"straight_offset", "heading_offset", "circle", "corner",
    "figure_eight", "campus_loop", "sharp_turns"};
}

bool makeScenario(const std::string & name, double v, Scenario & out)
{
  if (name == "straight_offset") {out = straightOffset(v); return true;}
  if (name == "heading_offset") {out = headingOffset(v); return true;}
  if (name == "circle") {out = circle(v); return true;}
  if (name == "corner") {out = corner(v); return true;}
  if (name == "figure_eight") {out = figureEight(v); return true;}
  if (name == "campus_loop") {out = campusLoop(v); return true;}
  if (name == "sharp_turns") {out = sharpTurns(v); return true;}
  return false;
}

std::vector<Scenario> makeScenarios(double v)
{
  std::vector<Scenario> out;
  for (const auto & name : scenarioNames()) {
    Scenario sc;
    if (makeScenario(name, v, sc)) {
      out.push_back(sc);
    }
  }
  return out;
}

}  // namespace shuttle
