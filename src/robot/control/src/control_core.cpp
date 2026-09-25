#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger,
                         double lookahead_distance,
                         double goal_tolerance,
                         double linear_speed,
                         double max_angular_speed)
  : logger_(logger),
    lookahead_distance_(lookahead_distance),
    goal_tolerance_(goal_tolerance),
    linear_speed_(linear_speed),
    max_angular_speed_(max_angular_speed)
{
}

geometry_msgs::msg::Twist ControlCore::computeCommand(
    const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom) const
{
  geometry_msgs::msg::Twist command;
  if (path.poses.empty()) {
    return command;
  }
  const auto& robot_position = odom.pose.pose.position;
  const auto& goal = path.poses.back().pose.position;
  if (distance(robot_position, goal) <= goal_tolerance_) {
    return command;
  }
  std::size_t closest_index = 0;
  double closest_distance = distance(robot_position, path.poses.front().pose.position);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    double candidate_distance = distance(robot_position, path.poses[i].pose.position);
    if (candidate_distance < closest_distance) {
      closest_distance = candidate_distance;
      closest_index = i;
    }
  }
  std::size_t target_index = path.poses.size() - 1;
  for (std::size_t i = closest_index; i < path.poses.size(); ++i) {
    if (distance(robot_position, path.poses[i].pose.position) >= lookahead_distance_) {
      target_index = i;
      break;
    }
  }
  const auto& target = path.poses[target_index].pose.position;
  double yaw = yawFromQuaternion(odom.pose.pose.orientation);
  double dx = target.x - robot_position.x;
  double dy = target.y - robot_position.y;
  double target_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  double target_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  double target_distance_squared = target_x * target_x + target_y * target_y;
  if (target_distance_squared < 1e-6) {
    return command;
  }
  double curvature = 2.0 * target_y / target_distance_squared;
  double heading_error = std::atan2(target_y, target_x);
  double speed_scale = std::max(0.2, 1.0 - std::abs(heading_error) / 3.14159265358979323846);
  command.linear.x = linear_speed_ * speed_scale;
  command.angular.z = std::clamp(command.linear.x * curvature,
                                 -max_angular_speed_, max_angular_speed_);
  return command;
}

double ControlCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlCore::distance(const geometry_msgs::msg::Point& a,
                             const geometry_msgs::msg::Point& b) const
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}
