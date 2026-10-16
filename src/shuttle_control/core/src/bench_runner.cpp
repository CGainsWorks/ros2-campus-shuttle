// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "shuttle_control/bench_runner.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>

namespace shuttle
{

std::uint32_t seedFor(const std::string & scenario, double speed, bool noise)
{
  const std::size_t h = std::hash<std::string>{}(
    scenario + "|" + std::to_string(static_cast<int>(speed * 100)) + "|" + (noise ? "n" : "c"));
  return static_cast<std::uint32_t>(h & 0xffffffffu) ^ 0x9e3779b9u;
}

RunResult runScenario(PathTracker & tracker, const Scenario & scenario, const RunConfig & config)
{
  RunResult result;
  if (scenario.path.size() < 2) {
    return result;
  }

  DiffDriveModel plant(config.plant, config.noise, config.seed);
  plant.reset(scenario.start, config.seed);

  tracker.setPath(scenario.path);
  // A benchmark run is a genuine restart, not a mid-flight replan, so clear the
  // speed limiter as well. setPath() deliberately preserves it (see the note
  // there), and the tuning sweep reuses one tracker across scenarios, so
  // without this a run would inherit the previous run's commanded speed and
  // results would depend on the order they were executed in.
  tracker.reset();

  const double control_dt = 1.0 / config.control_rate;
  const double physics_dt = 1.0 / config.physics_rate;
  const int substeps = std::max(1, static_cast<int>(std::lround(control_dt / physics_dt)));

  double t = 0.0;
  bool diverged = false;
  bool stalled = false;

  // Forward progress watchdog. A controller that sits on the path and spins
  // never diverges and never arrives, so without this the run just burns its
  // whole timeout. Treat "no arc-length progress for this long" as a failure.
  const double stall_timeout = 10.0;   // s
  const double stall_epsilon = 0.05;   // m
  double best_s = -1e9;
  double last_progress_t = 0.0;

  // The metrics keep their own projection state so that what is measured is
  // independent of whatever the controller believes internally.
  std::size_t metric_hint = 0;
  bool metric_started = false;

  // Sensing latency: the controller acts on a pose that is `latency_steps`
  // control periods old.
  std::deque<VehicleState> pipeline;
  const std::size_t latency =
    config.latency_steps > 0 ? static_cast<std::size_t>(config.latency_steps) : 0;

  while (t <= scenario.timeout) {
    const VehicleState fresh = plant.measure(t);
    pipeline.push_back(fresh);
    if (pipeline.size() > latency + 1) {
      pipeline.pop_front();
    }
    // Until the pipeline has filled, the oldest sample available is the best
    // estimate the controller could have had.
    const VehicleState measured = pipeline.front();

    const Twist2D cmd = tracker.computeCommand(measured, control_dt);

    // Error of the TRUE pose against the path.
    const Pose2D & truth = plant.truePose();
    const TrackingError truth_err =
      scenario.path.project(truth, metric_hint, metric_started ? 120 : 120);
    if (truth_err.valid) {
      metric_hint = truth_err.index;
      metric_started = true;
    }

    Sample s;
    s.t = t;
    s.x = truth.x;
    s.y = truth.y;
    s.yaw = truth.yaw;
    s.x_meas = measured.pose.x;
    s.y_meas = measured.pose.y;
    s.yaw_meas = measured.pose.yaw;
    s.path_x = truth_err.x;
    s.path_y = truth_err.y;
    s.s = truth_err.s;
    s.cross_track = truth_err.cross_track;
    s.heading = truth_err.heading;
    s.kappa = truth_err.kappa;
    s.v_cmd = cmd.v;
    s.omega_cmd = cmd.omega;
    s.v = plant.trueV();
    s.omega = plant.trueOmega();
    result.samples.push_back(s);

    if (std::fabs(truth_err.cross_track) > config.abort_cross_track) {
      diverged = true;
      break;
    }
    if (tracker.goalReached()) {
      break;
    }

    if (truth_err.s > best_s + stall_epsilon) {
      best_s = truth_err.s;
      last_progress_t = t;
    } else if (t - last_progress_t > stall_timeout) {
      stalled = true;
      break;
    }

    for (int i = 0; i < substeps; ++i) {
      plant.step(cmd, physics_dt);
    }
    t += control_dt;
  }

  result.metrics = summarise(
    result.samples,
    tracker.name(),
    scenario.name,
    scenario.path.empty() ? 0.0 : scenario.path.front().v_ref,
    config.noise.enabled,
    scenario.path.length(),
    tracker.goalReached(),
    diverged,
    stalled);
  return result;
}

}  // namespace shuttle
