#include "shuttle_control/diff_drive_model.hpp"

#include <algorithm>
#include <cmath>

namespace shuttle
{

void DiffDriveModel::reset(const Pose2D & pose, std::uint32_t seed)
{
  pose_ = pose;
  v_ = 0.0;
  omega_ = 0.0;
  yaw_bias_ = 0.0;
  rng_.seed(seed);
  normal_.reset();
}

double DiffDriveModel::gauss(double sigma)
{
  return sigma * normal_(rng_);
}

void DiffDriveModel::step(const Twist2D & cmd, double dt)
{
  if (dt <= 0.0) {
    return;
  }

  // Per-wheel saturation. A diff drive cannot honour an arbitrary (v, omega)
  // pair: it is limited by the faster wheel. Scale both down together so the
  // commanded arc is preserved rather than distorted.
  const double half_sep = 0.5 * params_.wheel_separation;
  double v_cmd = cmd.v;
  double w_cmd = cmd.omega;
  const double v_wheel_max = params_.max_wheel_speed * params_.wheel_radius;
  const double v_right = v_cmd + w_cmd * half_sep;
  const double v_left = v_cmd - w_cmd * half_sep;
  const double peak = std::max(std::fabs(v_right), std::fabs(v_left));
  if (peak > v_wheel_max && peak > 1e-9) {
    const double scale = v_wheel_max / peak;
    v_cmd *= scale;
    w_cmd *= scale;
  }

  // First-order lag towards the command, then an acceleration clamp.
  const double v_target = v_ + (v_cmd - v_) * std::min(1.0, dt / std::max(params_.tau_v, 1e-6));
  const double w_target =
    omega_ + (w_cmd - omega_) * std::min(1.0, dt / std::max(params_.tau_omega, 1e-6));

  const double dv = clampValue(v_target - v_, -params_.a_lon_max * dt, params_.a_lon_max * dt);
  const double dw = clampValue(w_target - omega_, -params_.alpha_max * dt, params_.alpha_max * dt);
  v_ += dv;
  omega_ += dw;

  // Exact arc integration of the unicycle model.
  if (std::fabs(omega_) > 1e-9) {
    const double yaw_next = pose_.yaw + omega_ * dt;
    const double r = v_ / omega_;
    pose_.x += r * (std::sin(yaw_next) - std::sin(pose_.yaw));
    pose_.y -= r * (std::cos(yaw_next) - std::cos(pose_.yaw));
    pose_.yaw = wrapToPi(yaw_next);
  } else {
    pose_.x += v_ * std::cos(pose_.yaw) * dt;
    pose_.y += v_ * std::sin(pose_.yaw) * dt;
  }

  if (noise_.enabled) {
    yaw_bias_ += gauss(noise_.yaw_bias_walk * std::sqrt(dt));
  }
}

VehicleState DiffDriveModel::measure(double t)
{
  VehicleState s;
  s.t = t;
  s.pose = pose_;
  s.v = v_;
  s.omega = omega_;
  if (noise_.enabled) {
    s.pose.x += gauss(noise_.sigma_xy);
    s.pose.y += gauss(noise_.sigma_xy);
    s.pose.yaw = wrapToPi(s.pose.yaw + yaw_bias_ + gauss(noise_.sigma_yaw));
    s.v += gauss(noise_.sigma_v);
    s.omega += gauss(noise_.sigma_omega);
  }
  return s;
}

}  // namespace shuttle
