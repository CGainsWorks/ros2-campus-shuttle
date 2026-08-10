#include "shuttle_control/nav2_shuttle_controllers.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
// Supplies the fromMsg() overload that tf2::getYaw() needs for a
// geometry_msgs quaternion. Without it tf2/utils.h compiles but does not link.
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace shuttle_control
{

using nav2_util::declare_parameter_if_not_declared;

void ShuttleControllerBase::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("ShuttleController: parent lifecycle node is gone");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, name + ".max_speed", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name + ".max_lon_accel", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, name + ".max_lat_accel", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, name + ".min_moving_speed", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, name + ".goal_tolerance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, name + ".max_omega", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, name + ".search_window", rclcpp::ParameterValue(120));
  declare_parameter_if_not_declared(
    node, name + ".control_period", rclcpp::ParameterValue(0.05));

  shuttle::TrackerCommon common;
  node->get_parameter(name + ".max_speed", common.speed.v_max);
  node->get_parameter(name + ".max_lon_accel", common.speed.a_lon_max);
  node->get_parameter(name + ".max_lat_accel", common.speed.a_lat_max);
  node->get_parameter(name + ".min_moving_speed", common.speed.v_min_moving);
  node->get_parameter(name + ".goal_tolerance", common.goal_tolerance);
  node->get_parameter(name + ".max_omega", common.max_omega);
  int window = 120;
  node->get_parameter(name + ".search_window", window);
  common.search_window = static_cast<std::size_t>(std::max(1, window));
  node->get_parameter(name + ".control_period", control_period_);

  max_speed_ = common.speed.v_max;

  tracker_ = makeTracker(node, name);
  tracker_->common() = common;
  tracker_->reset();

  error_pub_ = node->create_publisher<shuttle_msgs::msg::TrackingError>(
    "tracking_error", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(logger_, "configured %s", name.c_str());
}

void ShuttleControllerBase::cleanup()
{
  error_pub_.reset();
  tracker_.reset();
}

void ShuttleControllerBase::activate()
{
  if (error_pub_) {
    error_pub_->on_activate();
  }
}

void ShuttleControllerBase::deactivate()
{
  if (error_pub_) {
    error_pub_->on_deactivate();
  }
}

void ShuttleControllerBase::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    RCLCPP_WARN(logger_, "ignoring plan with %zu poses", path.poses.size());
    return;
  }
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(path.poses.size());
  ys.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    xs.push_back(p.pose.position.x);
    ys.push_back(p.pose.position.y);
  }
  tracker_->setPath(shuttle::Path::fromPolyline(xs, ys, max_speed_));
}

geometry_msgs::msg::TwistStamped ShuttleControllerBase::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  shuttle::VehicleState state;
  state.pose.x = pose.pose.position.x;
  state.pose.y = pose.pose.position.y;
  state.pose.yaw = tf2::getYaw(pose.pose.orientation);
  state.v = velocity.linear.x;
  state.omega = velocity.angular.z;
  state.t = clock_ ? clock_->now().seconds() : 0.0;

  const shuttle::Twist2D cmd = tracker_->computeCommand(state, control_period_);

  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = pose.header.frame_id;
  if (clock_) {
    out.header.stamp = clock_->now();
  } else {
    out.header.stamp = pose.header.stamp;
  }
  out.twist.linear.x = cmd.v;
  out.twist.angular.z = cmd.omega;

  if (error_pub_ && error_pub_->is_activated()) {
    const shuttle::TrackingError & err = tracker_->lastError();
    shuttle_msgs::msg::TrackingError msg;
    msg.header = out.header;
    msg.controller = tracker_->name();
    msg.cross_track = err.cross_track;
    msg.heading = err.heading;
    msg.arc_length = err.s;
    msg.curvature = err.kappa;
    msg.v_cmd = cmd.v;
    msg.omega_cmd = cmd.omega;
    msg.goal_reached = tracker_->goalReached();
    fillDiagnostics(msg);
    error_pub_->publish(msg);
  }

  return out;
}

void ShuttleControllerBase::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (!tracker_) {
    return;
  }
  auto & common = tracker_->common();
  if (speed_limit <= 0.0) {
    common.speed.v_max = max_speed_;  // no limit
  } else if (percentage) {
    common.speed.v_max = max_speed_ * speed_limit / 100.0;
  } else {
    common.speed.v_max = std::min(max_speed_, speed_limit);
  }
}

// --- Pure Pursuit -----------------------------------------------------------

std::unique_ptr<shuttle::PathTracker> PurePursuitController::makeTracker(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  declare_parameter_if_not_declared(
    node, name + ".lookahead_gain", rclcpp::ParameterValue(0.65));
  declare_parameter_if_not_declared(
    node, name + ".lookahead_base", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name + ".lookahead_min", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, name + ".lookahead_max", rclcpp::ParameterValue(3.0));

  shuttle::PurePursuitParams params;
  node->get_parameter(name + ".lookahead_gain", params.lookahead_gain);
  node->get_parameter(name + ".lookahead_base", params.lookahead_base);
  node->get_parameter(name + ".lookahead_min", params.lookahead_min);
  node->get_parameter(name + ".lookahead_max", params.lookahead_max);

  auto tracker = std::make_unique<shuttle::PurePursuit>(params);
  pp_ = tracker.get();
  return tracker;
}

void PurePursuitController::fillDiagnostics(shuttle_msgs::msg::TrackingError & msg) const
{
  if (pp_ != nullptr) {
    msg.lookahead = pp_->lastLookahead();
  }
}

// --- Stanley ----------------------------------------------------------------

std::unique_ptr<shuttle::PathTracker> StanleyController::makeTracker(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  declare_parameter_if_not_declared(
    node, name + ".k_e", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, name + ".k_soft", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, name + ".front_axle_distance", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, name + ".max_steer", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name + ".k_damp", rclcpp::ParameterValue(0.0));

  shuttle::StanleyParams params;
  node->get_parameter(name + ".k_e", params.k_e);
  node->get_parameter(name + ".k_soft", params.k_soft);
  node->get_parameter(name + ".front_axle_distance", params.front_axle_distance);
  node->get_parameter(name + ".max_steer", params.max_steer);
  node->get_parameter(name + ".k_damp", params.k_damp);

  auto tracker = std::make_unique<shuttle::Stanley>(params);
  stanley_ = tracker.get();
  return tracker;
}

void StanleyController::fillDiagnostics(shuttle_msgs::msg::TrackingError & msg) const
{
  if (stanley_ != nullptr) {
    msg.steer = stanley_->lastSteer();
    msg.front_axle_error = stanley_->lastFrontAxleError();
  }
}

}  // namespace shuttle_control

PLUGINLIB_EXPORT_CLASS(shuttle_control::PurePursuitController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(shuttle_control::StanleyController, nav2_core::Controller)
