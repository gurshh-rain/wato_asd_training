#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger,
                         double lookahead_distance,
                         double goal_tolerance,
                         double linear_speed,
                         double max_angular_speed,
                         double rotate_in_place_angle,
                         double angular_gain)
  : logger_(logger),
    lookahead_distance_(lookahead_distance),
    goal_tolerance_(goal_tolerance),
    linear_speed_(linear_speed),
    max_angular_speed_(max_angular_speed),
    rotate_in_place_angle_(rotate_in_place_angle),
    angular_gain_(angular_gain)
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
  double heading_error = std::atan2(target_y, target_x);
  if (std::abs(heading_error) >= rotate_in_place_angle_) {
    command.angular.z = std::clamp(angular_gain_ * heading_error,
                                   -max_angular_speed_, max_angular_speed_);
    return command;
  }
  double curvature = 2.0 * target_y / target_distance_squared;
  double heading_scale = std::max(0.25, 1.0 - std::abs(heading_error) / rotate_in_place_angle_);
  double curvature_scale = 1.0 / (1.0 + 2.5 * std::abs(curvature));
  command.linear.x = linear_speed_ * std::min(heading_scale, curvature_scale);
  command.angular.z = std::clamp(command.linear.x * curvature,
                                 -max_angular_speed_, max_angular_speed_);
  return command;
}

namespace
{
constexpr double kHorizon = 4.0;
constexpr double kStep = 0.05;
constexpr double kLidarOffset = 1.3;
constexpr double kMargin = 0.2;

geometry_msgs::msg::Point predictedPoint(const ObstaclePrediction& track, double time)
{
  double index = std::clamp((time + track.age) / track.step,
                            0.0, static_cast<double>(track.points.size() - 1));
  auto first = static_cast<std::size_t>(index);
  auto second = std::min(first + 1, track.points.size() - 1);
  double fraction = index - first;
  geometry_msgs::msg::Point point;
  point.x = track.points[first].x * (1.0 - fraction) + track.points[second].x * fraction;
  point.y = track.points[first].y * (1.0 - fraction) + track.points[second].y * fraction;
  return point;
}

void advance(double& x, double& y, double& yaw,
             const geometry_msgs::msg::Twist& command, double dt)
{
  double angle = command.angular.z * dt;
  if (std::abs(command.angular.z) > 1e-6) {
    x += command.linear.x / command.angular.z * (std::sin(yaw + angle) - std::sin(yaw));
    y += command.linear.x / command.angular.z * (std::cos(yaw) - std::cos(yaw + angle));
  } else {
    x += command.linear.x * dt * std::cos(yaw);
    y += command.linear.x * dt * std::sin(yaw);
  }
  yaw += angle;
}

double bodyClearance(double x, double y, double yaw,
                     const geometry_msgs::msg::Point& point, double radius)
{
  double dx = point.x - x;
  double dy = point.y - y;
  double forward = std::cos(yaw) * dx + std::sin(yaw) * dy;
  double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  double outside_x = std::max({-0.5 - forward, 0.0, forward - 1.5});
  double outside_y = std::max(0.0, std::abs(lateral) - 0.7);
  return std::hypot(outside_x, outside_y) - radius - kMargin;
}
}

MotionDecision ControlCore::selectCommand(
    const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom,
    const sensor_msgs::msg::LaserScan& scan, const std::vector<ObstaclePrediction>& predictions)
{
  auto nominal = computeCommand(path, odom);
  double nominal_clearance = motionClearance(nominal, odom, scan, predictions);
  if (nominal_clearance > 0.35) {
    last_turn_ = nominal.angular.z;
    return {nominal, path.poses.empty() ? "IDLE" : "FOLLOW_PATH", nominal_clearance};
  }
  const auto& robot = odom.pose.pose.position;
  geometry_msgs::msg::Point target = robot;
  if (!path.poses.empty()) {
    std::size_t closest = 0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      if (distance(robot, path.poses[i].pose.position) <
          distance(robot, path.poses[closest].pose.position)) {
        closest = i;
      }
    }
    double length = 0.0;
    for (std::size_t i = closest; i < path.poses.size(); ++i) {
      if (i > closest) {
        length += distance(path.poses[i - 1].pose.position, path.poses[i].pose.position);
      }
      target = path.poses[i].pose.position;
      if (length >= linear_speed_ * kHorizon) {
        break;
      }
    }
  }
  std::vector<geometry_msgs::msg::Twist> candidates{nominal};
  for (double speed : {linear_speed_, linear_speed_ * 0.65, linear_speed_ * 0.3, 0.0, -0.25}) {
    for (double turn : {0.0, 0.2, -0.2, 0.4, -0.4, 0.65, -0.65, 1.0, -1.0, 1.5, -1.5}) {
      if (std::abs(turn) > max_angular_speed_) {
        continue;
      }
      geometry_msgs::msg::Twist command;
      command.linear.x = speed;
      command.angular.z = turn;
      candidates.push_back(command);
    }
  }
  MotionDecision best;
  best.state = "NO_SAFE_MOTION";
  best.clearance = motionClearance(best.command, odom, scan, predictions);
  double best_score = -std::numeric_limits<double>::infinity();
  for (const auto& candidate : candidates) {
    double clearance = motionClearance(candidate, odom, scan, predictions);
    if (clearance <= 0.0) {
      continue;
    }
    double yaw = yawFromQuaternion(odom.pose.pose.orientation);
    double x = robot.x - kLidarOffset * std::cos(yaw);
    double y = robot.y - kLidarOffset * std::sin(yaw);
    advance(x, y, yaw, candidate, kHorizon);
    geometry_msgs::msg::Point endpoint;
    endpoint.x = x + kLidarOffset * std::cos(yaw);
    endpoint.y = y + kLidarOffset * std::sin(yaw);
    double progress = distance(robot, target) - distance(endpoint, target);
    double score = 1.5 * progress + 0.6 * std::min(clearance, 2.0) -
                   0.15 * std::abs(candidate.angular.z) -
                   0.25 * std::abs(candidate.angular.z - last_turn_);
    if (last_turn_ * candidate.angular.z < 0.0) {
      score -= 0.5;
    }
    if (candidate.linear.x < 0.0) {
      score -= 0.5;
    }
    if (score > best_score) {
      best_score = score;
      best.command = candidate;
      best.clearance = clearance;
      best.state = std::abs(candidate.linear.x) < 1e-6 &&
                   std::abs(candidate.angular.z) < 1e-6 ? "YIELD" : "AVOID";
    }
  }
  last_turn_ = best.command.angular.z;
  return best;
}

double ControlCore::motionClearance(
    const geometry_msgs::msg::Twist& command, const nav_msgs::msg::Odometry& odom,
    const sensor_msgs::msg::LaserScan& scan, const std::vector<ObstaclePrediction>& predictions) const
{
  double yaw = yawFromQuaternion(odom.pose.pose.orientation);
  const auto& origin = odom.pose.pose.position;
  if (scan.ranges.empty() || !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
      !std::isfinite(yaw) || !std::isfinite(command.linear.x) ||
      !std::isfinite(command.angular.z)) {
    return -std::numeric_limits<double>::infinity();
  }
  for (const auto& track : predictions) {
    if (track.points.empty() || !std::isfinite(track.step) || track.step <= 0.0 ||
        !std::isfinite(track.age) || track.age < 0.0 || track.age > 0.5 ||
        !std::isfinite(track.radius) || track.radius < 0.0) {
      return -std::numeric_limits<double>::infinity();
    }
    for (const auto& point : track.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return -std::numeric_limits<double>::infinity();
      }
    }
  }
  std::vector<std::pair<geometry_msgs::msg::Point, double>> static_points;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {
      continue;
    }
    double angle = yaw + scan.angle_min + i * scan.angle_increment;
    geometry_msgs::msg::Point point;
    point.x = origin.x + range * std::cos(angle);
    point.y = origin.y + range * std::sin(angle);
    bool dynamic = false;
    for (const auto& track : predictions) {
      if (distance(point, predictedPoint(track, 0.0)) <= track.radius + 0.25) {
        dynamic = true;
        break;
      }
    }
    if (!dynamic) {
      static_points.emplace_back(point,
          std::max(0.05, range * std::abs(scan.angle_increment) * 0.5));
    }
  }
  double x = origin.x - kLidarOffset * std::cos(yaw);
  double y = origin.y - kLidarOffset * std::sin(yaw);
  double clearance = std::numeric_limits<double>::infinity();
  for (int step = 0; step <= static_cast<int>(kHorizon / kStep); ++step) {
    double t = step * kStep;
    for (const auto& point : static_points) {
      clearance = std::min(clearance, bodyClearance(x, y, yaw, point.first, point.second));
    }
    for (const auto& track : predictions) {
      clearance = std::min(clearance, bodyClearance(
          x, y, yaw, predictedPoint(track, t), track.radius));
    }
    if (clearance <= 0.0) {
      return clearance;
    }
    advance(x, y, yaw, command, kStep);
  }
  return clearance;
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
