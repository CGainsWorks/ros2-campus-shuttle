// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <iostream>
#include <vector>

#include "shuttle_control/bench_runner.hpp"
#include "shuttle_control/command_gate.hpp"
#include "shuttle_control/diff_drive_model.hpp"
#include "shuttle_control/metrics.hpp"
#include "shuttle_control/path.hpp"
#include "shuttle_control/pure_pursuit.hpp"
#include "shuttle_control/scenarios.hpp"
#include "shuttle_control/stanley.hpp"
#include "test_framework.hpp"

using namespace shuttle;  // NOLINT

namespace
{

Path straightPath(double length, double v = 1.0)
{
  std::vector<double> xs;
  std::vector<double> ys;
  for (double x = 0.0; x <= length + 1e-9; x += 0.05) {
    xs.push_back(x);
    ys.push_back(0.0);
  }
  return Path::fromPolyline(xs, ys, v);
}

Path circlePath(double radius, double v = 1.0)
{
  std::vector<double> xs;
  std::vector<double> ys;
  const int n = 2000;
  for (int i = 0; i <= n; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) / n;
    xs.push_back(radius * std::cos(a));
    ys.push_back(radius * std::sin(a));
  }
  return Path::fromPolyline(xs, ys, v);
}

void testAngles()
{
  TEST_CASE("wrapToPi and angleDiff");
  CHECK_NEAR(wrapToPi(0.0), 0.0, 1e-12);
  CHECK_NEAR(wrapToPi(3.0 * kPi), kPi, 1e-9);
  CHECK_NEAR(wrapToPi(-3.0 * kPi), kPi, 1e-9);
  CHECK_NEAR(wrapToPi(kPi + 0.1), -kPi + 0.1, 1e-9);
  CHECK_NEAR(angleDiff(0.1, -0.1), 0.2, 1e-12);
  // Crossing the branch cut the short way round.
  CHECK_NEAR(angleDiff(-kPi + 0.1, kPi - 0.1), 0.2, 1e-9);
}

void testPathGeometry()
{
  TEST_CASE("path arc length, heading and curvature");
  const Path p = straightPath(10.0);
  CHECK_GT(p.size(), 100u);
  CHECK_NEAR(p.length(), 10.0, 1e-6);
  CHECK_NEAR(p.front().yaw, 0.0, 1e-9);
  CHECK_NEAR(p.back().yaw, 0.0, 1e-9);
  CHECK_NEAR(p.at(50).kappa, 0.0, 1e-6);

  const PathPoint mid = p.interpolate(5.0);
  CHECK_NEAR(mid.x, 5.0, 1e-6);
  CHECK_NEAR(mid.y, 0.0, 1e-9);

  // Clamping outside the extent.
  CHECK_NEAR(p.interpolate(-1.0).x, 0.0, 1e-9);
  CHECK_NEAR(p.interpolate(99.0).x, 10.0, 1e-6);

  TEST_CASE("curvature of a circle is 1/r with the turn direction as its sign");
  const double r = 4.0;
  const Path c = circlePath(r);
  // Sampled counter-clockwise, so curvature is positive (left turn).
  CHECK_NEAR(c.at(c.size() / 2).kappa, 1.0 / r, 2e-3);
  CHECK_NEAR(c.length(), 2.0 * kPi * r, 1e-2);
}

void testProjection()
{
  TEST_CASE("cross-track error is positive to the left of the path");
  const Path p = straightPath(10.0);

  const TrackingError left = p.project(Pose2D{5.0, 0.7, 0.0});
  CHECK(left.valid);
  CHECK_NEAR(left.cross_track, 0.7, 1e-6);
  CHECK_NEAR(left.s, 5.0, 1e-2);
  CHECK_NEAR(left.heading, 0.0, 1e-9);

  const TrackingError right = p.project(Pose2D{5.0, -0.7, 0.0});
  CHECK_NEAR(right.cross_track, -0.7, 1e-6);

  TEST_CASE("heading error is path heading minus robot heading");
  const TrackingError h = p.project(Pose2D{5.0, 0.0, 0.3});
  CHECK_NEAR(h.heading, -0.3, 1e-9);
}

void testSpeedLimiter()
{
  TEST_CASE("speed limiter respects the acceleration and curvature limits");
  SpeedLimits lim;
  lim.v_max = 2.0;
  lim.a_lon_max = 1.0;
  lim.a_lat_max = 0.5;
  SpeedLimiter sl(lim);

  // From rest, one 0.1 s step cannot exceed a_lon_max * dt.
  const double v1 = sl.compute(2.0, 0.0, 100.0, 0.1);
  CHECK_NEAR(v1, 0.1, 1e-9);

  // Given long enough on a straight, it saturates at v_max.
  double v = v1;
  for (int i = 0; i < 200; ++i) {
    v = sl.compute(2.0, 0.0, 100.0, 0.1);
  }
  CHECK_NEAR(v, 2.0, 1e-6);

  // A tight curve pulls it down to sqrt(a_lat / kappa).
  for (int i = 0; i < 200; ++i) {
    v = sl.compute(2.0, 1.0, 100.0, 0.1);
  }
  CHECK_NEAR(v, std::sqrt(0.5 / 1.0), 1e-3);

  TEST_CASE("speed limiter decelerates into the goal");
  SpeedLimiter sl2(lim);
  double v2 = 0.0;
  for (int i = 0; i < 500; ++i) {
    v2 = sl2.compute(2.0, 0.0, 0.05, 0.1);
  }
  CHECK(v2 <= std::sqrt(2.0 * lim.a_lon_max * 0.05) + 1e-6);
}

void testPlant()
{
  TEST_CASE("plant integrates a straight line and a circle");
  DiffDriveParams dp;
  dp.tau_v = 1e-6;  // remove the lag so the kinematics can be checked directly
  dp.tau_omega = 1e-6;
  dp.a_lon_max = 1e6;
  dp.alpha_max = 1e6;
  NoiseParams np;
  DiffDriveModel m(dp, np, 1);

  m.reset(Pose2D{0.0, 0.0, 0.0}, 1);
  for (int i = 0; i < 1000; ++i) {
    m.step(Twist2D{1.0, 0.0}, 0.001);
  }
  CHECK_NEAR(m.truePose().x, 1.0, 1e-6);
  CHECK_NEAR(m.truePose().y, 0.0, 1e-9);

  // v = 1, omega = 1 traces a unit circle; a quarter turn takes pi/2 s.
  m.reset(Pose2D{0.0, 0.0, 0.0}, 1);
  const int steps = static_cast<int>((kPi / 2.0) / 0.0001);
  for (int i = 0; i < steps; ++i) {
    m.step(Twist2D{1.0, 1.0}, 0.0001);
  }
  CHECK_NEAR(m.truePose().x, 1.0, 1e-3);
  CHECK_NEAR(m.truePose().y, 1.0, 1e-3);
  CHECK_NEAR(m.truePose().yaw, kPi / 2.0, 1e-3);

  TEST_CASE("wheel saturation scales v and omega together");
  DiffDriveParams sat;
  sat.max_wheel_speed = 1.0;   // 0.1 m/s at the wheel rim
  sat.wheel_radius = 0.1;
  sat.wheel_separation = 0.5;
  sat.tau_v = 1e-6;
  sat.tau_omega = 1e-6;
  sat.a_lon_max = 1e6;
  sat.alpha_max = 1e6;
  DiffDriveModel ms(sat, np, 1);
  ms.reset(Pose2D{0.0, 0.0, 0.0}, 1);
  ms.step(Twist2D{5.0, 0.0}, 0.01);
  CHECK(ms.trueV() <= 0.1 + 1e-9);

  TEST_CASE("noise is reproducible for a given seed");
  NoiseParams noisy;
  noisy.enabled = true;
  DiffDriveModel a(dp, noisy, 42);
  DiffDriveModel b(dp, noisy, 42);
  a.reset(Pose2D{}, 42);
  b.reset(Pose2D{}, 42);
  for (int i = 0; i < 20; ++i) {
    a.step(Twist2D{1.0, 0.1}, 0.05);
    b.step(Twist2D{1.0, 0.1}, 0.05);
    const VehicleState sa = a.measure(i * 0.05);
    const VehicleState sb = b.measure(i * 0.05);
    CHECK_NEAR(sa.pose.x, sb.pose.x, 1e-12);
    CHECK_NEAR(sa.pose.yaw, sb.pose.yaw, 1e-12);
  }
}

void testMetrics()
{
  TEST_CASE("error statistics");
  const std::vector<double> v{1.0, -2.0, 3.0, -4.0};
  const ErrorStats st = computeStats(v);
  CHECK_NEAR(st.mean_abs, 2.5, 1e-12);
  CHECK_NEAR(st.rms, std::sqrt((1.0 + 4.0 + 9.0 + 16.0) / 4.0), 1e-12);
  CHECK_NEAR(st.max_abs, 4.0, 1e-12);
  CHECK_NEAR(st.final_abs, 4.0, 1e-12);
  CHECK_EQ(st.count, 4u);
}

/// The steering-sign checks below are the ones that would catch a flipped
/// convention, which is the classic way to get a path tracker subtly wrong.
void testSteeringSigns()
{
  TEST_CASE("both controllers steer right when offset to the left");
  const Path p = straightPath(20.0);

  VehicleState s;
  s.pose = Pose2D{5.0, 1.0, 0.0};  // 1 m to the left of the path
  s.v = 1.0;

  PurePursuit pp;
  pp.common().speed.v_max = 1.0;
  pp.setPath(p);
  const Twist2D cmd_pp = pp.computeCommand(s, 0.05);
  CHECK(cmd_pp.omega < 0.0);

  Stanley st;
  st.common().speed.v_max = 1.0;
  st.setPath(p);
  const Twist2D cmd_st = st.computeCommand(s, 0.05);
  CHECK(cmd_st.omega < 0.0);

  TEST_CASE("both controllers steer left when offset to the right");
  s.pose = Pose2D{5.0, -1.0, 0.0};
  pp.reset();
  st.reset();
  CHECK(pp.computeCommand(s, 0.05).omega > 0.0);
  CHECK(st.computeCommand(s, 0.05).omega > 0.0);

  TEST_CASE("Stanley corrects a pure heading error with no lateral offset");
  s.pose = Pose2D{5.0, 0.0, 0.3};  // pointing left of the path
  st.reset();
  const Twist2D cmd_h = st.computeCommand(s, 0.05);
  CHECK(cmd_h.omega < 0.0);  // must turn back to the right
}

void testConvergence()
{
  TEST_CASE("both controllers converge from a lateral offset");
  for (int which = 0; which < 2; ++which) {
    Scenario sc;
    CHECK(makeScenario("straight_offset", 1.0, sc));

    RunConfig cfg;
    cfg.seed = seedFor("straight_offset", 1.0, false);

    PurePursuit pp;
    Stanley st;
    PathTracker * tracker = (which == 0) ?
      static_cast<PathTracker *>(&pp) :
      static_cast<PathTracker *>(&st);
    tracker->common().speed.v_max = 1.0;
    tracker->common().max_omega = 1.5;
    tracker->common().search_window = 120;

    const RunResult res = runScenario(*tracker, sc, cfg);
    std::cout << "    " << res.metrics.controller
              << ": final |e| = " << res.metrics.cross_track.final_abs
              << " m, RMS = " << res.metrics.cross_track.rms
              << " m, reached = " << res.metrics.goal_reached << "\n";
    CHECK(!res.metrics.diverged);
    CHECK(res.metrics.goal_reached);
    // Started 1 m off; must be settled on the line by the end of 30 m.
    CHECK(res.metrics.cross_track.final_abs < 0.05);
  }

  TEST_CASE("both controllers complete the campus loop");
  for (int which = 0; which < 2; ++which) {
    Scenario sc;
    CHECK(makeScenario("campus_loop", 1.0, sc));

    RunConfig cfg;
    cfg.seed = seedFor("campus_loop", 1.0, false);

    PurePursuit pp;
    Stanley st;
    PathTracker * tracker = (which == 0) ?
      static_cast<PathTracker *>(&pp) :
      static_cast<PathTracker *>(&st);
    tracker->common().speed.v_max = 1.0;
    tracker->common().search_window = 120;

    const RunResult res = runScenario(*tracker, sc, cfg);
    std::cout << "    " << res.metrics.controller
              << ": progress " << 100.0 * res.metrics.progress
              << "%, RMS |e| = " << res.metrics.cross_track.rms << " m\n";
    CHECK(!res.metrics.diverged);
    CHECK(res.metrics.progress > 0.99);
  }
}

/// Every reference path must be geometrically continuous.
///
/// This exists because a hand-chained version of the staircase scenario emitted
/// a path that jumped backwards by 1.15 m at one corner. The controllers then
/// stalled there and the failure looked exactly like a controller defect. A
/// comparison is only worth reporting if the reference trajectories are sound,
/// so they are checked rather than assumed.
void testPathContinuity()
{
  TEST_CASE("scenario paths are continuous and monotonic in arc length");
  for (const auto & name : scenarioNames()) {
    Scenario sc;
    CHECK(makeScenario(name, 1.0, sc));
    CHECK_GT(sc.path.size(), 10u);

    double worst_gap = 0.0;
    bool monotonic = true;
    for (std::size_t i = 1; i < sc.path.size(); ++i) {
      const double gap = std::hypot(
        sc.path.at(i).x - sc.path.at(i - 1).x,
        sc.path.at(i).y - sc.path.at(i - 1).y);
      worst_gap = std::max(worst_gap, gap);
      if (sc.path.at(i).s < sc.path.at(i - 1).s) {
        monotonic = false;
      }
    }
    // Paths are sampled at 0.05 m; allow a little slack for the final sample of
    // each primitive but nothing near a real discontinuity.
    if (worst_gap > 0.08) {
      std::cout << "    " << name << " has a " << worst_gap << " m gap\n";
    }
    CHECK(worst_gap <= 0.08);
    CHECK(monotonic);

    // The robot must start on or very near the path it is asked to follow.
    const TrackingError start = sc.path.project(sc.start);
    CHECK(start.valid);
    CHECK(std::fabs(start.cross_track) <= 1.01);
  }
}

/// Re-installing a plan must not throw away the speed the robot already has.
///
/// This is a regression test for a bug found only by running the stack in
/// Gazebo. The path publisher repeats its latched plan once a second and Nav2
/// republishes on every planner cycle. setPath() used to reset the speed
/// limiter, so the acceleration rate limit ramped the robot up from zero again
/// on every plan message. The commanded speed sawtoothed between 0 and 0.8 m/s
/// and the robot averaged a third of its target speed.
void testSetPathPreservesSpeed()
{
  TEST_CASE("re-setting the same plan does not reset the commanded speed");
  const Path p = straightPath(40.0, 1.0);

  PurePursuit tracker;
  tracker.common().speed.v_max = 1.0;
  tracker.common().speed.a_lon_max = 0.8;
  tracker.common().search_window = 120;
  tracker.setPath(p);

  VehicleState s;
  s.pose = Pose2D{0.0, 0.0, 0.0};
  s.v = 0.0;

  // Drive for two seconds so the speed limiter has ramped up.
  double v_cmd = 0.0;
  for (int i = 0; i < 40; ++i) {
    const Twist2D cmd = tracker.computeCommand(s, 0.05);
    v_cmd = cmd.v;
    s.pose.x += cmd.v * 0.05;
    s.v = cmd.v;
  }
  CHECK(v_cmd > 0.9);

  // The identical plan arrives again, as a latched republish would deliver it.
  tracker.setPath(p);
  const Twist2D after = tracker.computeCommand(s, 0.05);

  std::cout << "    before " << v_cmd << " m/s, after re-setPath "
            << after.v << " m/s\n";
  // One acceleration step of slack, no more. A reset would give ~0.04 m/s.
  CHECK(after.v > v_cmd - 0.8 * 0.05 - 1e-9);

  TEST_CASE("explicit reset() does clear the speed");
  tracker.reset();
  const Twist2D restarted = tracker.computeCommand(s, 0.05);
  CHECK(restarted.v <= 0.8 * 0.05 + 1e-9);
}

void testDeterminism()
{
  TEST_CASE("a run is bit-for-bit repeatable");
  Scenario sc;
  CHECK(makeScenario("circle", 1.0, sc));
  RunConfig cfg;
  cfg.noise.enabled = true;
  cfg.seed = seedFor("circle", 1.0, true);

  PurePursuit a;
  PurePursuit b;
  a.common().search_window = 120;
  b.common().search_window = 120;
  const RunResult ra = runScenario(a, sc, cfg);
  const RunResult rb = runScenario(b, sc, cfg);
  CHECK(ra.samples.size() == rb.samples.size());
  CHECK_NEAR(ra.metrics.cross_track.rms, rb.metrics.cross_track.rms, 1e-15);
}

void testCommandGate()
{
  TEST_CASE("command gate forwards fresh commands and zeroes stale ones");
  CommandGateParams params;
  params.cmd_timeout = 0.5;
  CommandGate gate(params);

  // Nothing received yet: the base is held at zero.
  auto held = gate.onTick(0.0);
  CHECK(held.has_value());
  CHECK_NEAR(held->v, 0.0, 0.0);
  CHECK(gate.state() == GateState::kNoCommand);

  Twist2D cmd;
  cmd.v = 1.0;
  cmd.omega = 0.3;
  const Twist2D out = gate.onCommand(cmd, 1.0);
  CHECK_NEAR(out.v, 1.0, 0.0);
  CHECK_NEAR(out.omega, 0.3, 0.0);
  // Fresh: the tick leaves the forwarded command alone.
  CHECK(!gate.onTick(1.4).has_value());
  CHECK(gate.state() == GateState::kForwarding);
  // Exactly at the timeout is still fresh; just past it is stale.
  CHECK(!gate.onTick(1.5).has_value());
  auto stale = gate.onTick(1.5001);
  CHECK(stale.has_value());
  CHECK_NEAR(stale->v, 0.0, 0.0);
  CHECK_NEAR(stale->omega, 0.0, 0.0);
  CHECK(gate.state() == GateState::kTimedOut);
  // A new command resumes forwarding at once.
  CHECK_NEAR(gate.onCommand(cmd, 2.0).v, 1.0, 0.0);
  CHECK(!gate.onTick(2.1).has_value());
  // A clock that runs backwards (simulator reset) is treated as stale.
  CHECK(gate.onTick(1.0).has_value());

  TEST_CASE("e-stop latches, drops commands and needs a fresh command after release");
  CommandGate estop(params);
  estop.onCommand(cmd, 0.0);
  CHECK(estop.engageEStop());
  CHECK(!estop.engageEStop());  // already engaged
  CHECK(estop.eStopped());
  const Twist2D dropped = estop.onCommand(cmd, 0.1);
  CHECK_NEAR(dropped.v, 0.0, 0.0);
  CHECK_NEAR(dropped.omega, 0.0, 0.0);
  auto stopped = estop.onTick(0.15);
  CHECK(stopped.has_value());
  CHECK_NEAR(stopped->v, 0.0, 0.0);
  CHECK(estop.state() == GateState::kEStopped);
  // Commands keep arriving during the stop; none of them survives it.
  for (int i = 0; i < 10; ++i) {
    estop.onCommand(cmd, 0.2 + 0.05 * i);
  }
  CHECK(estop.releaseEStop());
  CHECK(!estop.releaseEStop());  // already released
  auto after_release = estop.onTick(0.7);
  CHECK(after_release.has_value());
  CHECK_NEAR(after_release->v, 0.0, 0.0);
  CHECK(estop.state() == GateState::kNoCommand);
  CHECK_NEAR(estop.onCommand(cmd, 0.8).v, 1.0, 0.0);
  CHECK(!estop.onTick(0.85).has_value());
}

}  // namespace

int main()
{
  std::cout << "shuttle_control core tests\n";
  testAngles();
  testPathGeometry();
  testProjection();
  testSpeedLimiter();
  testPlant();
  testMetrics();
  testSteeringSigns();
  testPathContinuity();
  testSetPathPreservesSpeed();
  testConvergence();
  testDeterminism();
  testCommandGate();

  const auto & r = testing_lite::Registry::instance();
  std::cout << "\n" << r.checks << " checks, " << r.failures << " failures\n";
  return r.failures == 0 ? 0 : 1;
}
