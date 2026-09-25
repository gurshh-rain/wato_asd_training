#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

PlannerNode::PlannerNode()
  : Node("planner"),
    planner_(this->get_logger(),
             this->declare_parameter<int>("occupied_threshold", 65),
             this->declare_parameter<bool>("allow_diagonal", true)),
    state_(State::WAITING_FOR_GOAL),
    map_changed_(false),
    goal_tolerance_(this->declare_parameter<double>("goal_tolerance", 0.5)),
    progress_timeout_(this->declare_parameter<double>("progress_timeout", 3.0)),
    best_goal_distance_(std::numeric_limits<double>::infinity()),
    last_progress_time_(this->now())
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  double timer_rate = this->declare_parameter<double>("timer_rate", 2.0);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / timer_rate)),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  map_ = msg;
  map_changed_ = true;
}

void PlannerNode::goalCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
{
  goal_ = *msg;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  best_goal_distance_ = std::numeric_limits<double>::infinity();
  last_progress_time_ = this->now();
  planPath();
}

void PlannerNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void PlannerNode::timerCallback()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL || !odom_) {
    return;
  }
  double distance = goalDistance();
  if (distance <= goal_tolerance_) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = map_ ? map_->header.frame_id : goal_.header.frame_id;
    path_pub_->publish(empty_path);
    state_ = State::WAITING_FOR_GOAL;
    return;
  }
  if (distance + 0.05 < best_goal_distance_) {
    best_goal_distance_ = distance;
    last_progress_time_ = this->now();
  }
  bool timed_out = (this->now() - last_progress_time_).seconds() >= progress_timeout_;
  if (map_changed_ || timed_out) {
    planPath();
    last_progress_time_ = this->now();
  }
}

void PlannerNode::planPath()
{
  if (!map_ || !odom_ || state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }
  auto path = planner_.plan(*map_, odom_->pose.pose.position, goal_.point, this->now());
  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found to goal");
  }
  path_pub_->publish(path);
  map_changed_ = false;
}

double PlannerNode::goalDistance() const
{
  double dx = goal_.point.x - odom_->pose.pose.position.x;
  double dy = goal_.point.y - odom_->pose.pose.position.y;
  return std::hypot(dx, dy);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
