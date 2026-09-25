#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger,
                double lookahead_distance,
                double goal_tolerance,
                double linear_speed,
                double max_angular_speed);
    geometry_msgs::msg::Twist computeCommand(const nav_msgs::msg::Path& path,
                                             const nav_msgs::msg::Odometry& odom) const;

  private:
    rclcpp::Logger logger_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    double max_angular_speed_;
    double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const;
    double distance(const geometry_msgs::msg::Point& a,
                    const geometry_msgs::msg::Point& b) const;
};

}

#endif
