// Differential-drive plant model for the offline benchmark.
//
// This is not a replacement for Gazebo. It exists so the controller comparison
// can run deterministically, headlessly and in seconds, on any machine with a
// C++ compiler. The Gazebo runs (scripts/run_comparison.sh) exercise the same
// controller code against a physics engine and ground truth; this model
// exercises it against a plant whose behaviour is exactly known.
//
// Modelled effects, all of which act identically on both controllers:
//   - first-order actuator lag on v and omega
//   - acceleration limits
//   - per-wheel speed saturation (the real limit on a diff drive)
//   - exact arc integration of the unicycle kinematics
//   - optional seeded odometry noise: white noise plus a slow yaw bias
#pragma once

#include <cstdint>
#include <random>

#include "shuttle_control/types.hpp"

namespace shuttle
{

struct DiffDriveParams
{
  double wheel_separation{0.52};  ///< m
  double wheel_radius{0.10};      ///< m
  double max_wheel_speed{16.0};   ///< rad/s
  double tau_v{0.18};             ///< s, linear velocity lag
  double tau_omega{0.12};         ///< s, yaw rate lag
  double a_lon_max{1.2};          ///< m/s^2
  double alpha_max{4.0};          ///< rad/s^2
};

struct NoiseParams
{
  bool enabled{false};
  double sigma_xy{0.02};        ///< m, per-sample white noise on measured position
  double sigma_yaw{0.012};      ///< rad, per-sample white noise on measured heading
  double yaw_bias_walk{0.004};  ///< rad/sqrt(s), random-walk yaw bias
  double sigma_v{0.02};         ///< m/s, measured linear velocity
  double sigma_omega{0.02};     ///< rad/s, measured yaw rate
};

class DiffDriveModel
{
public:
  DiffDriveModel() = default;
  DiffDriveModel(const DiffDriveParams & params, const NoiseParams & noise, std::uint32_t seed)
  : params_(params), noise_(noise), rng_(seed) {}

  void setParams(const DiffDriveParams & p) { params_ = p; }
  void setNoise(const NoiseParams & n) { noise_ = n; }

  /// Reset to a pose. The RNG is re-seeded so that a given (scenario, speed,
  /// noise) combination produces the same disturbance sequence for every
  /// controller under test.
  void reset(const Pose2D & pose, std::uint32_t seed);

  /// Advance the plant by dt under command `cmd`.
  void step(const Twist2D & cmd, double dt);

  /// True state (the Gazebo-ground-truth analogue). Used only for metrics.
  const Pose2D & truePose() const { return pose_; }
  double trueV() const { return v_; }
  double trueOmega() const { return omega_; }

  /// Noisy measurement handed to the controller.
  VehicleState measure(double t);

  const DiffDriveParams & params() const { return params_; }

private:
  double gauss(double sigma);

  DiffDriveParams params_;
  NoiseParams noise_;
  std::mt19937 rng_{0};
  std::normal_distribution<double> normal_{0.0, 1.0};

  Pose2D pose_;
  double v_{0.0};
  double omega_{0.0};
  double yaw_bias_{0.0};
};

}  // namespace shuttle
