#include "control_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>

ControlNode::ControlNode()
  : Node("control"),
    control_(this->get_logger(),
             this->declare_parameter<double>("lookahead_distance", 0.4),
             this->declare_parameter<double>("goal_tolerance", 0.2),
             this->declare_parameter<double>("linear_speed", 1.0),
             this->declare_parameter<double>("max_angular_speed", 1.5),
             this->declare_parameter<double>("rotate_in_place_angle", 0.4),
             this->declare_parameter<double>("angular_gain", 1.8)),
    data_timeout_(this->declare_parameter<double>("data_timeout", 1.0)),
    prediction_step_(this->declare_parameter<double>("prediction_step", 0.2))
{
  auto new_goal = [this](geometry_msgs::msg::PointStamped::ConstSharedPtr) {
    guide_.reset();
    path_.reset();
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  };
  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, new_goal);
  clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, new_goal);
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 1, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 1, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ControlNode::scanCallback, this, std::placeholders::_1));
  tracks_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/tracked_obstacles", 1,
      std::bind(&ControlNode::tracksCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/control_status", 1);
  double control_rate = this->declare_parameter<double>("control_rate", 10.0);
  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / control_rate),
                                  std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  path_ = msg;
  path_received_ = std::chrono::steady_clock::now();
  if (path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  } else {
    guide_ = path_;
  }
}

void ControlNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
  odom_received_ = std::chrono::steady_clock::now();
}

void ControlNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  scan_ = msg;
  scan_received_ = std::chrono::steady_clock::now();
}

void ControlNode::tracksCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
{
  tracks_ = msg;
  tracks_received_ = std::chrono::steady_clock::now();
}

void ControlNode::controlLoop()
{
  if (!path_) {
    return;
  }
  auto now = std::chrono::steady_clock::now();
  auto stale = [&](auto received) {
    return std::chrono::duration<double>(now - received).count() > data_timeout_;
  };
  robot::MotionDecision decision;
  decision.state = "STALE_DATA";
  bool fresh = odom_ && scan_ && tracks_ && !stale(odom_received_) &&
      !stale(scan_received_) && !stale(tracks_received_) &&
      (path_->poses.empty() || !stale(path_received_));
  if (fresh) {
    auto seconds = [](const builtin_interfaces::msg::Time& stamp) {
      return stamp.sec + stamp.nanosec * 1e-9;
    };
    double reference = std::max(seconds(scan_->header.stamp), seconds(odom_->header.stamp));
    bool valid = path_->poses.empty() || path_->header.frame_id == odom_->header.frame_id;
    valid = valid && std::abs(seconds(scan_->header.stamp) - seconds(odom_->header.stamp)) <= 0.25;
    std::vector<robot::ObstaclePrediction> predictions;
    for (const auto& marker : tracks_->markers) {
      if (marker.ns != "predictions" || marker.action != marker.ADD) {
        continue;
      }
      robot::ObstaclePrediction track;
      track.id = marker.id;
      track.points = marker.points;
      track.step = prediction_step_;
      if (!marker.text.empty()) {
        std::istringstream stream(marker.text);
        stream >> track.step;
        valid = valid && !stream.fail();
      }
      track.age = reference - seconds(marker.header.stamp);
      valid = valid && track.age >= -0.1 && track.age <= 0.5 &&
              marker.header.frame_id == odom_->header.frame_id;
      track.age = std::max(0.0, track.age);
      for (const auto& body : tracks_->markers) {
        if (body.ns == "tracked_obstacles" && body.id + 1 == marker.id) {
          track.radius = std::max(0.8, body.scale.x * 0.5 + 0.5);
        }
      }
      predictions.push_back(track);
    }
    if (valid) {
      auto route = path_;
      if (path_->poses.empty() && guide_ && !stale(path_received_)) {
        const auto& goal = guide_->poses.back().pose.position;
        const auto& position = odom_->pose.pose.position;
        if (std::hypot(goal.x - position.x, goal.y - position.y) > 0.5) {
          route = guide_;
        } else {
          guide_.reset();
        }
      }
      decision = control_.selectCommand(*route, *odom_, *scan_, predictions);
    } else {
      decision.state = "INVALID_PREDICTION_OR_FRAME";
    }
  }
  cmd_vel_pub_->publish(decision.command);
  std_msgs::msg::String status;
  status.data = decision.state;
  status_pub_->publish(status);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
