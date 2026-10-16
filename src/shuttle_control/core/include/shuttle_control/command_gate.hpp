// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Command gate: the software safety layer between every velocity source and
// the base.
//
// Two jobs, both of which end in a zero command:
//
//   * a command-timeout watchdog. A base that keeps executing the last velocity
//     it was sent after the sender has died is the "stale command" hazard in
//     docs/SAFETY.md. The simulated differential drive does exactly that, as a
//     real motor controller without a firmware timeout would, so the gate stops
//     forwarding and outputs zero once no fresh command has arrived within
//     `cmd_timeout` seconds.
//
//   * a latched emergency stop. Once engaged, every incoming command is dropped
//     and the output is zero until the stop is explicitly released. Releasing
//     does not resume the last command: motion restarts only when a new command
//     arrives after the release, so an operator clearing a stop never gets a
//     robot that lurches off on a command issued before the stop.
//
// Time is passed in rather than read from a clock, so the same object runs on
// the simulation clock in ROS and on synthetic time in the unit tests. Like the
// rest of core/, it depends on nothing but the standard library.
#pragma once

#include <optional>

#include "shuttle_control/types.hpp"

namespace shuttle
{

struct CommandGateParams
{
  /// A command older than this is stale and the output is forced to zero, s.
  double cmd_timeout{0.5};
};

enum class GateState
{
  kNoCommand,   ///< nothing received yet, or nothing since an e-stop release
  kForwarding,  ///< a fresh command is being passed through
  kTimedOut,    ///< the last command is older than cmd_timeout
  kEStopped,    ///< the emergency stop is latched
};

const char * toString(GateState state);

class CommandGate
{
public:
  explicit CommandGate(CommandGateParams params = {});

  /// Offer a command received at time `t`. Returns what the base should be
  /// sent now: the command itself, or zero while the e-stop is latched.
  Twist2D onCommand(const Twist2D & cmd, double t);

  /// Periodic check at time `t`. Returns a zero command whenever the base must
  /// be held stopped (no command, stale command, e-stop). Returns nothing while
  /// fresh commands are being forwarded, so the sender sets the command rate.
  std::optional<Twist2D> onTick(double t);

  /// Latch the emergency stop. Returns true if it was not already engaged.
  bool engageEStop();

  /// Release the emergency stop. Motion resumes only on the next command.
  /// Returns true if it was engaged.
  bool releaseEStop();

  bool eStopped() const {return e_stopped_;}
  GateState state() const {return state_;}
  const CommandGateParams & params() const {return params_;}

private:
  CommandGateParams params_;
  GateState state_{GateState::kNoCommand};
  bool e_stopped_{false};
  bool have_command_{false};
  double last_command_time_{0.0};
};

}  // namespace shuttle
