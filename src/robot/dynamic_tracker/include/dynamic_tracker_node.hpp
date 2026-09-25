#ifndef DYNAMIC_TRACKER_NODE_HPP_
#define DYNAMIC_TRACKER_NODE_HPP_

#include "dynamic_tracker_core.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class DynamicTrackerNode : public rclcpp::Node {
  public:
    DynamicTrackerNode();

  private:
    robot::DynamicTrackerCore tracker_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr pending_scan_;
    void processScan();
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
};

#endif
