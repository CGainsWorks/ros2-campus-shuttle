// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Tracking-error metrics.
//
// Every quantity here is computed from the *true* pose (the plant state
// offline, the Gazebo ground-truth pose on the robot), never from the pose the
// controller was given. A controller that tracks its own noisy estimate
// beautifully while drifting off the real path should score badly, and this is
// what makes that happen.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "shuttle_control/types.hpp"

namespace shuttle
{

/// Summary statistics of one scalar error signal.
struct ErrorStats
{
  double mean_abs{0.0};
  double rms{0.0};
  double max_abs{0.0};
  double p95_abs{0.0};
  double final_abs{0.0};
  std::size_t count{0};
};

ErrorStats computeStats(const std::vector<double> & values);

/// One logged control step.
struct Sample
{
  double t{0.0};
  double x{0.0};            ///< true pose
  double y{0.0};
  double yaw{0.0};
  double x_meas{0.0};       ///< pose the controller saw
  double y_meas{0.0};
  double yaw_meas{0.0};
  double path_x{0.0};       ///< projection of the true pose onto the path
  double path_y{0.0};
  double s{0.0};
  double cross_track{0.0};  ///< signed, left-positive, at base_link
  double heading{0.0};      ///< signed, rad
  double v_cmd{0.0};
  double omega_cmd{0.0};
  double v{0.0};
  double omega{0.0};
  double kappa{0.0};
};

/// Aggregate result of a single run.
struct RunMetrics
{
  std::string controller;
  std::string scenario;
  double target_speed{0.0};
  bool noise{false};

  ErrorStats cross_track;
  ErrorStats heading;

  double duration{0.0};            ///< s
  double path_length{0.0};         ///< m
  double distance_travelled{0.0};  ///< m, true path integral
  double progress{0.0};            ///< fraction of path arc length completed, 0..1
  bool goal_reached{false};
  bool diverged{false};            ///< |cross-track| exceeded the abort threshold
  /// Stopped making forward progress along the path while still on it. This is
  /// how Pure Pursuit fails with too small a lookahead: it stays on the line
  /// and spins, so it neither diverges nor arrives.
  bool stalled{false};

  double mean_speed{0.0};          ///< m/s
  double control_effort{0.0};      ///< RMS of d(omega_cmd)/dt, rad/s^2
  double rms_omega{0.0};           ///< rad/s
  double max_lateral_accel{0.0};   ///< m/s^2
};

/// Compute the aggregate metrics for a completed run.
RunMetrics summarise(
  const std::vector<Sample> & samples,
  const std::string & controller,
  const std::string & scenario,
  double target_speed,
  bool noise,
  double path_length,
  bool goal_reached,
  bool diverged,
  bool stalled);

/// CSV helpers, shared by the offline bench and the Gazebo metrics recorder so
/// both feed the same analysis script.
std::string sampleCsvHeader();
std::string sampleCsvRow(const Sample & s);
std::string metricsCsvHeader();
std::string metricsCsvRow(const RunMetrics & m);

}  // namespace shuttle
