// Nav2 controller plugins wrapping the shuttle path trackers.
//
// Exposing the controllers as nav2_core::Controller plugins means Nav2 can
// drive them through its normal behaviour tree, with its planner, costmaps and
// goal checkers. The plugins hold a tracker from shuttle_control_core and do no
// control maths of their own, so what Nav2 runs is the same code the benchmark
// measures.
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "shuttle_control/path_tracker.hpp"
#include "shuttle_control/pure_pursuit.hpp"
#include "shuttle_control/stanley.hpp"
#include "shuttle_msgs/msg/tracking_error.hpp"
#include "tf2_ros/buffer.h"

namespace shuttle_control
{

/// Shared plumbing for both plugins.
class ShuttleControllerBase : public nav2_core::Controller
{
public:
  ShuttleControllerBase() = default;
  ~ShuttleControllerBase() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /// Build the tracker and read its controller-specific parameters.
  virtual std::unique_ptr<shuttle::PathTracker> makeTracker(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name) = 0;

  /// Fill in the controller-specific diagnostic fields.
  virtual void fillDiagnostics(shuttle_msgs::msg::TrackingError & msg) const {(void)msg;}

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("ShuttleController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<shuttle::PathTracker> tracker_;
  double max_speed_{1.0};
  double control_period_{0.05};

  rclcpp_lifecycle::LifecyclePublisher<shuttle_msgs::msg::TrackingError>::SharedPtr
    error_pub_;
};

class PurePursuitController : public ShuttleControllerBase
{
protected:
  std::unique_ptr<shuttle::PathTracker> makeTracker(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name) override;
  void fillDiagnostics(shuttle_msgs::msg::TrackingError & msg) const override;

private:
  const shuttle::PurePursuit * pp_{nullptr};
};

class StanleyController : public ShuttleControllerBase
{
protected:
  std::unique_ptr<shuttle::PathTracker> makeTracker(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name) override;
  void fillDiagnostics(shuttle_msgs::msg::TrackingError & msg) const override;

private:
  const shuttle::Stanley * stanley_{nullptr};
};

}  // namespace shuttle_control
