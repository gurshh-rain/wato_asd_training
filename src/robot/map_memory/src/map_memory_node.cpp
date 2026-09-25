#include <chrono>
#include <functional>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"),
    map_memory_(robot::MapMemoryCore(
        this->get_logger(),
        this->declare_parameter<double>("resolution", 0.05),
        this->declare_parameter<int>("width", 600),
        this->declare_parameter<int>("height", 600),
        this->declare_parameter<double>("origin_x", -15.0),
        this->declare_parameter<double>("origin_y", -15.0),
        this->declare_parameter<std::string>("frame_id", "sim_world"),
        this->declare_parameter<double>("distance_threshold", 0.25)))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered",
      10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  double update_rate = this->declare_parameter<double>("update_rate", 5.0);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
      std::bind(&MapMemoryNode::timerCallback, this));
  map_pub_->publish(map_memory_.generateMap(this->now()));
}

void MapMemoryNode::costmapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  pending_costmap_ = msg;
}

void MapMemoryNode::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  map_memory_.updateOdom(msg);
}

void MapMemoryNode::timerCallback()
{
  if (pending_costmap_) {
    try {
      auto transform = tf_buffer_->lookupTransform(get_parameter("frame_id").as_string(),
          pending_costmap_->header.frame_id, rclcpp::Time(pending_costmap_->header.stamp));
      auto pose = std::make_shared<nav_msgs::msg::Odometry>();
      pose->pose.pose.position.x = transform.transform.translation.x;
      pose->pose.pose.position.y = transform.transform.translation.y;
      pose->pose.pose.orientation = transform.transform.rotation;
      map_memory_.updateOdom(pose);
      map_memory_.updateCostmap(pending_costmap_);
      pending_costmap_.reset();
    } catch (const tf2::TransformException&) {
    }
  }
  map_pub_->publish(map_memory_.generateMap(this->now()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
