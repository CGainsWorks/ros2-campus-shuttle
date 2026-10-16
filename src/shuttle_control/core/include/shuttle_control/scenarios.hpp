// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Reference trajectories for the controller comparison.
//
// The same six paths are driven by both controllers at every speed and noise
// setting, from the same start pose, with the same seed. They are chosen to
// separate the two failure modes the comparison is about: geometric corner
// cutting (Pure Pursuit) and heading-noise sensitivity (Stanley).
#pragma once

#include <string>
#include <vector>

#include "shuttle_control/path.hpp"
#include "shuttle_control/types.hpp"

namespace shuttle
{

struct Scenario
{
  std::string name;
  std::string description;
  Path path;
  Pose2D start;           ///< initial true pose of the robot
  double timeout{240.0};  ///< s, abort if the goal is not reached
};

/// Build every scenario at the given nominal speed.
std::vector<Scenario> makeScenarios(double nominal_speed);

/// Build one scenario by name; `name` must be one of the names returned above.
/// Returns false if the name is unknown.
bool makeScenario(const std::string & name, double nominal_speed, Scenario & out);

/// Names of all scenarios, in a stable order.
std::vector<std::string> scenarioNames();

}  // namespace shuttle
