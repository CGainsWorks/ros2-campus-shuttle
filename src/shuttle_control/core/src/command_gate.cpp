// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "shuttle_control/command_gate.hpp"

namespace shuttle
{

const char * toString(GateState state)
{
  switch (state) {
    case GateState::kNoCommand:
      return "no_command";
    case GateState::kForwarding:
      return "forwarding";
    case GateState::kTimedOut:
      return "timed_out";
    case GateState::kEStopped:
      return "e_stopped";
  }
  return "unknown";
}

CommandGate::CommandGate(CommandGateParams params)
: params_(params)
{
}

Twist2D CommandGate::onCommand(const Twist2D & cmd, double t)
{
  if (e_stopped_) {
    // Dropped, not queued: nothing sent during a stop may act after it.
    return Twist2D{};
  }
  have_command_ = true;
  last_command_time_ = t;
  state_ = GateState::kForwarding;
  return cmd;
}

std::optional<Twist2D> CommandGate::onTick(double t)
{
  if (e_stopped_) {
    state_ = GateState::kEStopped;
    return Twist2D{};
  }
  if (!have_command_) {
    state_ = GateState::kNoCommand;
    return Twist2D{};
  }
  // A clock that has gone backwards (a simulator reset) makes the age of the
  // last command meaningless, so treat it as stale rather than as fresh.
  const double age = t - last_command_time_;
  if (age < 0.0 || age > params_.cmd_timeout) {
    state_ = GateState::kTimedOut;
    return Twist2D{};
  }
  state_ = GateState::kForwarding;
  return std::nullopt;
}

bool CommandGate::engageEStop()
{
  const bool changed = !e_stopped_;
  e_stopped_ = true;
  state_ = GateState::kEStopped;
  return changed;
}

bool CommandGate::releaseEStop()
{
  const bool changed = e_stopped_;
  e_stopped_ = false;
  have_command_ = false;
  state_ = GateState::kNoCommand;
  return changed;
}

}  // namespace shuttle
