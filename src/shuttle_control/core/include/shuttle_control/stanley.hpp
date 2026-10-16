// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Stanley path tracker, adapted to a differential-drive base.
//
// The Stanley law (Stanford's DARPA Grand Challenge entry) is defined for a
// front-wheel-steered vehicle. It regulates the cross-track error measured at
// the *front axle* and outputs a steering angle:
//
//     delta = psi + atan2(k_e * e_fa, k_soft + v)
//
// where psi is the heading error and e_fa the front-axle cross-track error.
//
// A differential drive has no front axle and no steering angle, so two things
// have to be supplied to make the law meaningful:
//
//   1. A *virtual front axle* a distance L_fa ahead of base_link. The error
//      e_fa is measured there rather than at the robot origin.
//   2. A conversion from steering angle to yaw rate, using the bicycle
//      relation omega = v * tan(delta) / L_fa.
//
// L_fa therefore plays the role of a wheelbase and is a genuine tuning
// parameter of this adaptation, not a property of the robot. This is the main
// caveat on the comparison and is discussed in docs/CONTROLLERS.md.
//
// Sign convention: `cross_track` is positive when the robot is to the LEFT of
// the path (see types.hpp), so the feedback term is negated relative to the
// textbook form, which defines the error the other way round.
#pragma once

#include "shuttle_control/path_tracker.hpp"

namespace shuttle
{

struct StanleyParams
{
  double k_e{2.0};                ///< cross-track gain
  double k_soft{0.5};             ///< softening constant, m/s (avoids blow-up at v -> 0)
  double front_axle_distance{0.45};  ///< L_fa, m
  double max_steer{1.0};          ///< rad, clamp on delta
  /// Optional yaw-rate damping (Stanley's later "yaw damping" extension).
  /// Zero reproduces the textbook law, which is the default so the comparison
  /// is against the published controller rather than a local variant.
  double k_damp{0.0};
};

class Stanley : public PathTracker
{
public:
  Stanley() = default;
  explicit Stanley(const StanleyParams & params)
  : params_(params) {}

  const char * name() const override {return "stanley";}

  StanleyParams & params() {return params_;}
  const StanleyParams & params() const {return params_;}

  Twist2D computeCommand(const VehicleState & state, double dt) override;

  /// Steering angle commanded on the most recent call, rad.
  double lastSteer() const {return last_steer_;}
  /// Cross-track error at the virtual front axle on the most recent call, m.
  double lastFrontAxleError() const {return last_fa_error_;}

private:
  StanleyParams params_;
  double last_steer_{0.0};
  double last_fa_error_{0.0};
};

}  // namespace shuttle
