// Copyright 2026 CGainsWorks
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Safety gate between every velocity source and the base.
//
//   cmd_vel_in  (geometry_msgs/Twist)  path tracker, Nav2 velocity smoother,
//                                       teleop: anything that wants to move
//   cmd_vel     (geometry_msgs/Twist)  the only topic the base listens to
//
//   e_stop      (std_msgs/Bool)        true latches the stop, false releases
//   set_e_stop  (std_srvs/SetBool)     the same, with a reply
//   e_stop/active (std_msgs/Bool)      latched state, transient local
//
// The decisions are made by shuttle::CommandGate in the ROS-free core, which
// is unit tested on synthetic time. This node only moves messages and owns the
// clock. It runs on the ROS clock, so in simulation the timeout is measured in
// simulated seconds: the time the robot actually experiences.
//
// A stop is acted on immediately, not at the next tick: engaging the e-stop
// publishes a zero command from inside the callback and the periodic tick then
// keeps publishing zero for as long as the stop is held, so a source that
// bypasses this gate cannot hold the base in motion either.
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shuttle_control/command_gate.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace shuttle
{

class SafetyGateNode : public rclcpp::Node
{
public:
  SafetyGateNode()
  : rclcpp::Node("safety_gate")
  {
    CommandGateParams params;
    params.cmd_timeout = declare_parameter<double>("cmd_timeout", 0.5);
    const double rate = declare_parameter<double>("rate", 50.0);
    const bool start_stopped = declare_parameter<bool>("start_e_stopped", false);
    gate_ = CommandGate(params);
    if (start_stopped) {
      gate_.engageEStop();
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::Bool>(
      "e_stop/active", rclcpp::QoS(1).transient_local());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 10,
      std::bind(&SafetyGateNode::onCommand, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "e_stop", rclcpp::QoS(10).reliable(),
      std::bind(&SafetyGateNode::onEStopTopic, this, std::placeholders::_1));
    estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "set_e_stop",
      std::bind(
        &SafetyGateNode::onEStopService, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      std::bind(&SafetyGateNode::onTick, this));

    publishState();
    RCLCPP_INFO(
      get_logger(), "safety gate up: command timeout %.3f s, %.0f Hz, e-stop %s",
      params.cmd_timeout, rate, gate_.eStopped() ? "ENGAGED" : "released");
  }

private:
  static geometry_msgs::msg::Twist toMsg(const Twist2D & cmd)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = cmd.v;
    msg.angular.z = cmd.omega;
    return msg;
  }

  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    Twist2D cmd;
    cmd.v = msg->linear.x;
    cmd.omega = msg->angular.z;
    cmd_pub_->publish(toMsg(gate_.onCommand(cmd, now().seconds())));
    logTransition();
  }

  void onTick()
  {
    const auto out = gate_.onTick(now().seconds());
    if (out) {
      cmd_pub_->publish(toMsg(*out));
    }
    logTransition();
  }

  void setEStop(bool engage, const std::string & source)
  {
    const bool changed = engage ? gate_.engageEStop() : gate_.releaseEStop();
    if (engage) {
      // Act now rather than at the next tick.
      cmd_pub_->publish(toMsg(Twist2D{}));
    }
    if (changed) {
      if (engage) {
        RCLCPP_WARN(get_logger(), "E-STOP ENGAGED (%s)", source.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(), "e-stop released (%s); waiting for a fresh command",
          source.c_str());
      }
      publishState();
    }
    logTransition();
  }

  void onEStopTopic(const std_msgs::msg::Bool::SharedPtr msg)
  {
    setEStop(msg->data, "topic");
  }

  void onEStopService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    setEStop(request->data, "service");
    response->success = true;
    response->message = gate_.eStopped() ? "e-stop engaged" : "e-stop released";
  }

  void publishState()
  {
    std_msgs::msg::Bool msg;
    msg.data = gate_.eStopped();
    state_pub_->publish(msg);
  }

  void logTransition()
  {
    if (gate_.state() == last_logged_) {
      return;
    }
    if (gate_.state() == GateState::kTimedOut) {
      RCLCPP_WARN(
        get_logger(), "no command for %.3f s: holding the base at zero",
        gate_.params().cmd_timeout);
    } else {
      RCLCPP_INFO(get_logger(), "gate state: %s", toString(gate_.state()));
    }
    last_logged_ = gate_.state();
  }

  CommandGate gate_;
  GateState last_logged_{GateState::kNoCommand};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace shuttle

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<shuttle::SafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
