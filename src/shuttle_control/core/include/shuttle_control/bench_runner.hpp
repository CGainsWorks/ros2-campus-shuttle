// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Closed-loop runner used by the offline comparison and by the unit tests.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "shuttle_control/diff_drive_model.hpp"
#include "shuttle_control/metrics.hpp"
#include "shuttle_control/path_tracker.hpp"
#include "shuttle_control/scenarios.hpp"

namespace shuttle
{

struct RunConfig
{
  double control_rate{20.0};       ///< Hz, matches the ROS 2 controller rate
  double physics_rate{200.0};      ///< Hz, plant integration
  double abort_cross_track{6.0};   ///< m, declare divergence and stop

  /// Sensing-to-actuation latency, in control steps.
  ///
  /// The pose a real controller acts on is already old: it has been through
  /// scan matching, the EKF and the ROS transport. Two steps at 20 Hz is 100 ms,
  /// which is representative of an AMCL/EKF pipeline.
  ///
  /// This is not a detail. With a latency-free plant the tuner drives the
  /// Pure Pursuit lookahead down to 0.15 m, because in a perfect world a
  /// shorter lookahead simply tracks better. Latency is what makes an
  /// unreasonably short lookahead oscillate, so without it the tuning step
  /// selects gains that would be unusable on a real robot.
  int latency_steps{2};

  DiffDriveParams plant;
  NoiseParams noise;
  std::uint32_t seed{1};
};

struct RunResult
{
  RunMetrics metrics;
  std::vector<Sample> samples;
};

/// Derive the RNG seed from the run conditions only, deliberately excluding the
/// controller: both controllers must face the identical noise realisation for
/// the comparison to be like-for-like.
std::uint32_t seedFor(const std::string & scenario, double speed, bool noise);

/// Drive `tracker` around `scenario` and return the metrics and the time series.
RunResult runScenario(PathTracker & tracker, const Scenario & scenario, const RunConfig & config);

}  // namespace shuttle
