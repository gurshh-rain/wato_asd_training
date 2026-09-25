#include "planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>

PlannerNode::PlannerNode()
  : Node("planner"),
    planner_(this->get_logger(),
             this->declare_parameter<int>("occupied_threshold", 1),
             this->declare_parameter<bool>("allow_diagonal", true)),
    fallback_planner_(this->get_logger(),
                      this->get_parameter("occupied_threshold").as_int(),
                      this->get_parameter("allow_diagonal").as_bool()),
    state_(State::WAITING_FOR_GOAL),
    map_changed_(false),
    goal_tolerance_(this->declare_parameter<double>("goal_tolerance", 0.5)),
    progress_timeout_(this->declare_parameter<double>("progress_timeout", 3.0)),
    footprint_radius_(this->declare_parameter<double>("footprint_radius", 1.2)),
    start_clearance_radius_(this->declare_parameter<double>("start_clearance_radius", 0.5)),
    best_goal_distance_(std::numeric_limits<double>::infinity()),
    last_progress_time_(this->now())
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  dynamic_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/dynamic_costmap", 10,
      std::bind(&PlannerNode::dynamicCostmapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  fallback_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/planner_dynamic_fallback", 10);
  double timer_rate = this->declare_parameter<double>("timer_rate", 5.0);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / timer_rate)),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  map_ = msg;
  map_changed_ = true;
}

void PlannerNode::dynamicCostmapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  dynamic_costmap_ = msg;
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
    std_msgs::msg::Bool fallback;
    fallback.data = false;
    fallback_pub_->publish(fallback);
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
  nav_msgs::msg::OccupancyGrid static_map = *map_;
  applyFootprint(static_map);
  clearStartRegion(static_map, *map_);
  nav_msgs::msg::OccupancyGrid planning_map = static_map;
  bool has_dynamic_layer = dynamic_costmap_ &&
      dynamic_costmap_->info.width == planning_map.info.width &&
      dynamic_costmap_->info.height == planning_map.info.height &&
      dynamic_costmap_->data.size() == planning_map.data.size();
  if (has_dynamic_layer) {
    for (std::size_t i = 0; i < planning_map.data.size(); ++i) {
      planning_map.data[i] = std::max(planning_map.data[i], dynamic_costmap_->data[i]);
    }
  }
  auto path = planner_.plan(planning_map, odom_->pose.pose.position, goal_.point, this->now());
  bool using_fallback = false;
  if (path.poses.empty() && has_dynamic_layer) {
    path = fallback_planner_.plan(
        static_map, odom_->pose.pose.position, goal_.point, this->now());
    using_fallback = !path.poses.empty();
  }
  std_msgs::msg::Bool fallback;
  fallback.data = using_fallback;
  fallback_pub_->publish(fallback);
  if (path.poses.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No static or dynamic path found to goal");
  } else if (using_fallback) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Dynamic route blocked; using timed-yield fallback");
  }
  path_pub_->publish(path);
  map_changed_ = false;
}

void PlannerNode::applyFootprint(nav_msgs::msg::OccupancyGrid& map) const
{
  if (map.info.resolution <= 0.0 || footprint_radius_ <= 0.0) {
    return;
  }
  auto expanded = map.data;
  int width = static_cast<int>(map.info.width);
  int height = static_cast<int>(map.info.height);
  int radius_cells = static_cast<int>(std::ceil(footprint_radius_ / map.info.resolution));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (map.data[static_cast<std::size_t>(y) * map.info.width +
                   static_cast<std::size_t>(x)] < 100) {
        continue;
      }
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
                  map.info.resolution > footprint_radius_) {
            continue;
          }
          int nx = x + dx;
          int ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
            expanded[static_cast<std::size_t>(ny) * map.info.width +
                     static_cast<std::size_t>(nx)] = 100;
          }
        }
      }
    }
  }
  map.data = std::move(expanded);
}

void PlannerNode::clearStartRegion(nav_msgs::msg::OccupancyGrid& map,
                                   const nav_msgs::msg::OccupancyGrid& original) const
{
  if (!odom_ || map.info.resolution <= 0.0 || start_clearance_radius_ <= 0.0) {
    return;
  }
  int width = static_cast<int>(map.info.width);
  int height = static_cast<int>(map.info.height);
  int center_x = static_cast<int>((odom_->pose.pose.position.x -
      map.info.origin.position.x) / map.info.resolution);
  int center_y = static_cast<int>((odom_->pose.pose.position.y -
      map.info.origin.position.y) / map.info.resolution);
  int radius_cells = static_cast<int>(std::ceil(start_clearance_radius_ /
                                                 map.info.resolution));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
              map.info.resolution > start_clearance_radius_) {
        continue;
      }
      int x = center_x + dx;
      int y = center_y + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      std::size_t index = static_cast<std::size_t>(y) * map.info.width +
                          static_cast<std::size_t>(x);
      if (original.data[index] < 100) {
        map.data[index] = 0;
      }
    }
  }
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
