#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <chrono>
#include <vector>

#include "control_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_, clicked_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path::ConstSharedPtr path_, guide_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
    visualization_msgs::msg::MarkerArray::ConstSharedPtr tracks_;
    std::chrono::steady_clock::time_point scan_received_, odom_received_, tracks_received_, path_received_;
    double data_timeout_;
    double prediction_step_;
    void pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void tracksCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
    void controlLoop();
};

#endif
