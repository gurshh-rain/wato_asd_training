#include "dynamic_tracker_node.hpp"

#include <functional>
#include <memory>

DynamicTrackerNode::DynamicTrackerNode()
  : Node("dynamic_tracker"),
    tracker_(this->get_logger(),
             this->declare_parameter<double>("cluster_threshold", 0.6),
             this->declare_parameter<int>("minimum_cluster_size", 2),
             this->declare_parameter<double>("maximum_cluster_radius", 0.9),
             this->declare_parameter<double>("association_gate", 1.2),
             this->declare_parameter<double>("dynamic_speed_threshold", 0.2),
             this->declare_parameter<double>("process_noise", 0.6),
             this->declare_parameter<double>("measurement_noise", 0.12),
             this->declare_parameter<double>("prediction_horizon", 5.0),
             this->declare_parameter<double>("occupancy_horizon", 3.0),
             this->declare_parameter<double>("prediction_step", 0.2),
             this->declare_parameter<double>("robot_radius", 0.9),
             this->declare_parameter<double>("safety_margin", 0.2),
             this->declare_parameter<double>("motion_min_x", -12.0),
             this->declare_parameter<double>("motion_max_x", 12.0))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", rclcpp::SensorDataQoS(),
      std::bind(&DynamicTrackerNode::scanCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&DynamicTrackerNode::odomCallback, this, std::placeholders::_1));
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&DynamicTrackerNode::mapCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/dynamic_costmap", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tracked_obstacles", 10);
}

void DynamicTrackerNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  pending_scan_ = msg;
  processScan();
}

void DynamicTrackerNode::processScan()
{
  if (!pending_scan_ || !map_) {
    return;
  }
  nav_msgs::msg::Odometry pose;
  auto stamp = rclcpp::Time(pending_scan_->header.stamp);
  try {
    auto transform = tf_buffer_->lookupTransform(
        map_->header.frame_id, pending_scan_->header.frame_id, stamp);
    pose.header = transform.header;
    pose.pose.pose.position.x = transform.transform.translation.x;
    pose.pose.pose.position.y = transform.transform.translation.y;
    pose.pose.pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException&) {
    return;
  }
  auto output = tracker_.process(*pending_scan_, pose, *map_, stamp);
  pending_scan_.reset();
  costmap_pub_->publish(output.dynamic_costmap);
  marker_pub_->publish(output.markers);
}

void DynamicTrackerNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
  processScan();
}

void DynamicTrackerNode::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  map_ = msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
