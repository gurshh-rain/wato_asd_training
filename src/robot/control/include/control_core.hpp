#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>

namespace robot
{

struct ObstaclePrediction {
  int id = 0;
  double radius = 0.8;
  double step = 0.2;
  double age = 0.0;
  std::vector<geometry_msgs::msg::Point> points;
};

struct MotionDecision {
  geometry_msgs::msg::Twist command;
  std::string state;
  double clearance = 0.0;
};

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger,
                double lookahead_distance,
                double goal_tolerance,
                double linear_speed,
                double max_angular_speed,
                double rotate_in_place_angle,
                double angular_gain);
    geometry_msgs::msg::Twist computeCommand(const nav_msgs::msg::Path& path,
                                             const nav_msgs::msg::Odometry& odom) const;
    MotionDecision selectCommand(const nav_msgs::msg::Path& path,
                                 const nav_msgs::msg::Odometry& odom,
                                 const sensor_msgs::msg::LaserScan& scan,
                                 const std::vector<ObstaclePrediction>& predictions);
    double motionClearance(const geometry_msgs::msg::Twist& command,
                           const nav_msgs::msg::Odometry& odom,
                           const sensor_msgs::msg::LaserScan& scan,
                           const std::vector<ObstaclePrediction>& predictions) const;

  private:
    double last_turn_ = 0.0;
    rclcpp::Logger logger_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    double max_angular_speed_;
    double rotate_in_place_angle_;
    double angular_gain_;
    double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const;
    double distance(const geometry_msgs::msg::Point& a,
                    const geometry_msgs::msg::Point& b) const;
};

}

#endif
