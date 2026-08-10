// Standalone ROS 2 path-tracking node.
//
// Wraps the controllers from shuttle_control_core. The node is deliberately
// thin: it converts ROS messages into the core's plain types, calls
// computeCommand(), and publishes the result. All the control logic lives in
// the core library, which is the same code the offline benchmark and the Nav2
// plugins run, so a result measured in one place holds in the others.
//
// The comparison harness uses this node rather than Nav2 so that both
// controllers see an identical, pre-recorded plan instead of a plan that a
// planner might regenerate differently between runs.
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shuttle_control/pure_pursuit.hpp"
#include "shuttle_control/stanley.hpp"
#include "shuttle_msgs/msg/tracking_error.hpp"
#include "tf2/utils.h"
// Supplies the fromMsg() overload that tf2::getYaw() needs for a
// geometry_msgs quaternion. Without it tf2/utils.h compiles but does not link.
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace shuttle
{

class PathTrackerNode : public rclcpp::Node
{
public:
  PathTrackerNode()
  : rclcpp::Node("path_tracker")
  {
    controller_type_ = declare_parameter<std::string>("controller_type", "pure_pursuit");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    control_rate_ = declare_parameter<double>("control_rate", 20.0);

    TrackerCommon common;
    common.speed.v_max = declare_parameter<double>("max_speed", 1.0);
    common.speed.a_lon_max = declare_parameter<double>("max_lon_accel", 0.8);
    common.speed.a_lat_max = declare_parameter<double>("max_lat_accel", 0.8);
    common.speed.v_min_moving = declare_parameter<double>("min_moving_speed", 0.15);
    common.goal_tolerance = declare_parameter<double>("goal_tolerance", 0.25);
    common.max_omega = declare_parameter<double>("max_omega", 1.5);
    common.search_window =
      static_cast<std::size_t>(declare_parameter<int>("search_window", 120));

    PurePursuitParams pp;
    pp.lookahead_gain = declare_parameter<double>("pure_pursuit.lookahead_gain", 0.65);
    pp.lookahead_base = declare_parameter<double>("pure_pursuit.lookahead_base", 0.0);
    pp.lookahead_min = declare_parameter<double>("pure_pursuit.lookahead_min", 0.15);
    pp.lookahead_max = declare_parameter<double>("pure_pursuit.lookahead_max", 3.0);

    StanleyParams st;
    st.k_e = declare_parameter<double>("stanley.k_e", 1.5);
    st.k_soft = declare_parameter<double>("stanley.k_soft", 0.05);
    st.front_axle_distance =
      declare_parameter<double>("stanley.front_axle_distance", 0.3);
    st.max_steer = declare_parameter<double>("stanley.max_steer", 1.0);

    if (controller_type_ == "stanley") {
      auto tracker = std::make_unique<Stanley>(st);
      stanley_ = tracker.get();
      tracker_ = std::move(tracker);
    } else if (controller_type_ == "pure_pursuit") {
      auto tracker = std::make_unique<PurePursuit>(pp);
      pure_pursuit_ = tracker.get();
      tracker_ = std::move(tracker);
    } else {
      RCLCPP_FATAL(
        get_logger(), "unknown controller_type '%s' (expected pure_pursuit or stanley)",
        controller_type_.c_str());
      throw std::runtime_error("unknown controller_type");
    }
    tracker_->common() = common;
    tracker_->reset();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    error_pub_ = create_publisher<shuttle_msgs::msg::TrackingError>("tracking_error", 10);

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1).transient_local(),
      std::bind(&PathTrackerNode::onPlan, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 20, std::bind(&PathTrackerNode::onOdom, this, std::placeholders::_1));

    // A ROS-clock timer, not a wall timer. Under use_sim_time these differ:
    // create_wall_timer always ticks on steady time, so if Gazebo runs faster
    // or slower than real time the control rate no longer matches the rate the
    // robot actually experiences, and the loop is effectively open.
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / control_rate_),
      std::bind(&PathTrackerNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "path_tracker up, controller=%s, %.1f Hz",
      controller_type_.c_str(), control_rate_);
  }

private:
  void onPlan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "ignoring plan with %zu poses", msg->poses.size());
      return;
    }
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(msg->poses.size());
    ys.reserve(msg->poses.size());
    for (const auto & p : msg->poses) {
      xs.push_back(p.pose.position.x);
      ys.push_back(p.pose.position.y);
    }
    // A latched plan is redelivered on every reconnection and the comparison
    // publisher repeats it once a second. Re-installing an identical path would
    // churn the tracker's state for no reason, so ignore repeats.
    if (have_plan_ && msg->poses.size() == last_plan_size_ &&
      std::fabs(xs.front() - last_plan_first_x_) < 1e-9 &&
      std::fabs(ys.front() - last_plan_first_y_) < 1e-9 &&
      std::fabs(xs.back() - last_plan_last_x_) < 1e-9 &&
      std::fabs(ys.back() - last_plan_last_y_) < 1e-9)
    {
      return;
    }
    last_plan_size_ = msg->poses.size();
    last_plan_first_x_ = xs.front();
    last_plan_first_y_ = ys.front();
    last_plan_last_x_ = xs.back();
    last_plan_last_y_ = ys.back();

    tracker_->setPath(Path::fromPolyline(xs, ys, tracker_->common().speed.v_max));
    have_plan_ = true;
    RCLCPP_INFO(
      get_logger(), "received plan: %zu poses, %.1f m",
      msg->poses.size(), tracker_->path().length());
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    measured_v_ = msg->twist.twist.linear.x;
    measured_omega_ = msg->twist.twist.angular.z;
    have_odom_ = true;
  }

  /// Look up the robot pose from TF rather than trusting the odometry frame,
  /// so the controller runs against the localised pose once AMCL or the global
  /// EKF is up.
  bool currentPose(Pose2D & out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "no %s -> %s transform: %s",
        global_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
    out.x = tf.transform.translation.x;
    out.y = tf.transform.translation.y;
    out.yaw = tf2::getYaw(tf.transform.rotation);
    return true;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

  void onTimer()
  {
    if (!have_plan_) {
      return;
    }
    Pose2D pose;
    if (!currentPose(pose)) {
      publishStop();
      return;
    }

    VehicleState state;
    state.pose = pose;
    state.v = have_odom_ ? measured_v_ : 0.0;
    state.omega = have_odom_ ? measured_omega_ : 0.0;
    state.t = now().seconds();

    const double dt = 1.0 / control_rate_;
    const Twist2D cmd = tracker_->computeCommand(state, dt);

    geometry_msgs::msg::Twist out;
    out.linear.x = cmd.v;
    out.angular.z = cmd.omega;
    cmd_pub_->publish(out);

    const TrackingError & err = tracker_->lastError();
    shuttle_msgs::msg::TrackingError msg;
    msg.header.stamp = now();
    msg.header.frame_id = global_frame_;
    msg.controller = controller_type_;
    msg.cross_track = err.cross_track;
    msg.heading = err.heading;
    msg.arc_length = err.s;
    msg.curvature = err.kappa;
    msg.v_cmd = cmd.v;
    msg.omega_cmd = cmd.omega;
    msg.goal_reached = tracker_->goalReached();
    if (pure_pursuit_ != nullptr) {
      msg.lookahead = pure_pursuit_->lastLookahead();
    }
    if (stanley_ != nullptr) {
      msg.steer = stanley_->lastSteer();
      msg.front_axle_error = stanley_->lastFrontAxleError();
    }
    error_pub_->publish(msg);

    if (tracker_->goalReached() && !announced_goal_) {
      announced_goal_ = true;
      RCLCPP_INFO(get_logger(), "goal reached");
    }
  }

  std::string controller_type_;
  std::string global_frame_;
  std::string base_frame_;
  double control_rate_{20.0};

  std::unique_ptr<PathTracker> tracker_;
  PurePursuit * pure_pursuit_{nullptr};
  Stanley * stanley_{nullptr};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<shuttle_msgs::msg::TrackingError>::SharedPtr error_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_plan_{false};
  std::size_t last_plan_size_{0};
  double last_plan_first_x_{0.0};
  double last_plan_first_y_{0.0};
  double last_plan_last_x_{0.0};
  double last_plan_last_y_{0.0};
  bool have_odom_{false};
  bool announced_goal_{false};
  double measured_v_{0.0};
  double measured_omega_{0.0};
};

}  // namespace shuttle

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<shuttle::PathTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
