#include "control_node.hpp"

#include <chrono>
#include <functional>
#include <memory>

ControlNode::ControlNode()
  : Node("control"),
    control_(this->get_logger(),
             this->declare_parameter<double>("lookahead_distance", 1.0),
             this->declare_parameter<double>("goal_tolerance", 0.2),
             this->declare_parameter<double>("linear_speed", 0.5),
             this->declare_parameter<double>("max_angular_speed", 1.5))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  double control_rate = this->declare_parameter<double>("control_rate", 10.0);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate)),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  path_ = msg;
  if (path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }
}

void ControlNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void ControlNode::controlLoop()
{
  if (!path_ || path_->poses.empty() || !odom_) {
    return;
  }
  cmd_vel_pub_->publish(control_.computeCommand(*path_, *odom_));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
