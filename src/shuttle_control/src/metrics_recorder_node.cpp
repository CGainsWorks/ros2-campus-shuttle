// Records tracking metrics for a Gazebo run.
//
// Deliberately written in C++ against shuttle_control_core rather than as a
// Python node with its own geometry: it reuses Path::project() and summarise(),
// so a Gazebo run and an offline benchmark run are scored by literally the same
// code and emit byte-compatible CSV. Both then feed
// analysis/compare_controllers.py without a translation step.
//
// Error is computed from /ground_truth/odom, which the Gazebo p3d plugin fills
// straight from the physics engine. Nothing here ever looks at the estimated
// pose the controller was driving on.
#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shuttle_control/metrics.hpp"
#include "shuttle_control/path.hpp"
#include "shuttle_msgs/msg/controller_metrics.hpp"
#include "shuttle_msgs/msg/tracking_error.hpp"
#include "tf2/utils.h"
// Supplies the fromMsg() overload that tf2::getYaw() needs for a
// geometry_msgs quaternion. Without it tf2/utils.h compiles but does not link.
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace shuttle
{

class MetricsRecorderNode : public rclcpp::Node
{
public:
  MetricsRecorderNode()
  : rclcpp::Node("metrics_recorder")
  {
    controller_ = declare_parameter<std::string>("controller", "unknown");
    scenario_ = declare_parameter<std::string>("scenario", "gazebo");
    target_speed_ = declare_parameter<double>("target_speed", 1.0);
    output_dir_ = declare_parameter<std::string>("output_dir", "results/gazebo");
    settle_timeout_ = declare_parameter<double>("settle_timeout", 5.0);

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1).transient_local(),
      std::bind(&MetricsRecorderNode::onPlan, this, std::placeholders::_1));
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "ground_truth/odom", 50,
      std::bind(&MetricsRecorderNode::onTruth, this, std::placeholders::_1));
    error_sub_ = create_subscription<shuttle_msgs::msg::TrackingError>(
      "tracking_error", 20,
      std::bind(&MetricsRecorderNode::onError, this, std::placeholders::_1));

    metrics_pub_ = create_publisher<shuttle_msgs::msg::ControllerMetrics>(
      "controller_metrics", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(
      get_logger(), "recording %s on %s, writing to %s",
      controller_.c_str(), scenario_.c_str(), output_dir_.c_str());
  }

  /// Write the results. Called from main() on shutdown so the files land even
  /// when the run is stopped with Ctrl-C.
  void finish()
  {
    if (wrote_) {
      return;
    }
    wrote_ = true;

    if (samples_.empty()) {
      RCLCPP_WARN(get_logger(), "no samples recorded, nothing written");
      return;
    }

    const RunMetrics m = summarise(
      samples_, controller_, scenario_, target_speed_, /*noise=*/true,
      path_.length(), goal_reached_, /*diverged=*/false, /*stalled=*/false);

    const std::string tag = controller_ + "_" + scenario_;
    const std::string ts_path = output_dir_ + "/timeseries/" + tag + "_gazebo.csv";
    std::ofstream ts(ts_path);
    if (ts) {
      ts << sampleCsvHeader() << "\n";
      for (const auto & s : samples_) {
        ts << sampleCsvRow(s) << "\n";
      }
      RCLCPP_INFO(get_logger(), "wrote %s", ts_path.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "cannot write %s (does %s/timeseries exist?)",
        ts_path.c_str(), output_dir_.c_str());
    }

    // Append to a shared metrics file, writing the header only if new.
    const std::string metrics_path = output_dir_ + "/metrics.csv";
    const bool exists = std::ifstream(metrics_path).good();
    std::ofstream mf(metrics_path, std::ios::app);
    if (mf) {
      if (!exists) {
        mf << metricsCsvHeader() << "\n";
      }
      mf << metricsCsvRow(m) << "\n";
      RCLCPP_INFO(get_logger(), "appended to %s", metrics_path.c_str());
    }

    shuttle_msgs::msg::ControllerMetrics msg;
    msg.header.stamp = now();
    msg.controller = m.controller;
    msg.scenario = m.scenario;
    msg.target_speed = m.target_speed;
    msg.noise = m.noise;
    msg.ct_mean_abs = m.cross_track.mean_abs;
    msg.ct_rms = m.cross_track.rms;
    msg.ct_max_abs = m.cross_track.max_abs;
    msg.ct_p95_abs = m.cross_track.p95_abs;
    msg.ct_final_abs = m.cross_track.final_abs;
    msg.hd_mean_abs = m.heading.mean_abs;
    msg.hd_rms = m.heading.rms;
    msg.hd_max_abs = m.heading.max_abs;
    msg.hd_p95_abs = m.heading.p95_abs;
    msg.hd_final_abs = m.heading.final_abs;
    msg.duration = m.duration;
    msg.path_length = m.path_length;
    msg.distance_travelled = m.distance_travelled;
    msg.progress = m.progress;
    msg.goal_reached = m.goal_reached;
    msg.diverged = m.diverged;
    msg.stalled = m.stalled;
    msg.mean_speed = m.mean_speed;
    msg.control_effort = m.control_effort;
    msg.rms_omega = m.rms_omega;
    msg.max_lateral_accel = m.max_lateral_accel;
    msg.samples = static_cast<uint32_t>(m.cross_track.count);
    metrics_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "%s: RMS cross-track %.4f m, max %.4f m, RMS heading %.4f rad, progress %.1f%%",
      controller_.c_str(), m.cross_track.rms, m.cross_track.max_abs,
      m.heading.rms, 100.0 * m.progress);
  }

  bool done() const
  {
    return goal_reached_ &&
           (now().seconds() - goal_time_) > settle_timeout_;
  }

private:
  void onPlan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      return;
    }
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto & p : msg->poses) {
      xs.push_back(p.pose.position.x);
      ys.push_back(p.pose.position.y);
    }
    path_ = Path::fromPolyline(xs, ys, target_speed_);
    RCLCPP_INFO(get_logger(), "plan: %zu poses, %.1f m", msg->poses.size(), path_.length());
  }

  void onError(const shuttle_msgs::msg::TrackingError::SharedPtr msg)
  {
    last_v_cmd_ = msg->v_cmd;
    last_omega_cmd_ = msg->omega_cmd;
    if (msg->goal_reached && !goal_reached_) {
      goal_reached_ = true;
      goal_time_ = now().seconds();
      RCLCPP_INFO(get_logger(), "controller reported the goal reached");
    }
  }

  void onTruth(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (path_.size() < 2) {
      return;  // nothing to measure against yet
    }
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    if (!started_) {
      start_time_ = t;
      started_ = true;
    }

    Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf2::getYaw(msg->pose.pose.orientation);

    const TrackingError err = path_.project(pose, hint_, 150);
    if (!err.valid) {
      return;
    }
    hint_ = err.index;

    Sample s;
    s.t = t - start_time_;
    s.x = pose.x;
    s.y = pose.y;
    s.yaw = pose.yaw;
    s.x_meas = pose.x;  // ground truth run: no separate estimate logged here
    s.y_meas = pose.y;
    s.yaw_meas = pose.yaw;
    s.path_x = err.x;
    s.path_y = err.y;
    s.s = err.s;
    s.cross_track = err.cross_track;
    s.heading = err.heading;
    s.kappa = err.kappa;
    s.v_cmd = last_v_cmd_;
    s.omega_cmd = last_omega_cmd_;
    s.v = msg->twist.twist.linear.x;
    s.omega = msg->twist.twist.angular.z;
    samples_.push_back(s);
  }

  std::string controller_;
  std::string scenario_;
  std::string output_dir_;
  double target_speed_{1.0};
  double settle_timeout_{5.0};

  Path path_;
  std::vector<Sample> samples_;
  std::size_t hint_{0};
  bool started_{false};
  double start_time_{0.0};
  bool goal_reached_{false};
  double goal_time_{0.0};
  bool wrote_{false};
  double last_v_cmd_{0.0};
  double last_omega_cmd_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;
  rclcpp::Subscription<shuttle_msgs::msg::TrackingError>::SharedPtr error_sub_;
  rclcpp::Publisher<shuttle_msgs::msg::ControllerMetrics>::SharedPtr metrics_pub_;
};

}  // namespace shuttle

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<shuttle::MetricsRecorderNode>();

  rclcpp::Rate rate(20.0);
  while (rclcpp::ok() && !node->done()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  node->finish();

  rclcpp::shutdown();
  return 0;
}
