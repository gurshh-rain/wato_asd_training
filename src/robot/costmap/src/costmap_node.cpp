#include <chrono>
#include <functional>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
  : Node("costmap"),
    costmap_(robot::CostmapCore(
        this->get_logger(),
        this->declare_parameter<double>("resolution", 0.1),
        this->declare_parameter<double>("grid_size", 20.0),
        this->declare_parameter<double>("inflation_radius", 0.75),
        this->declare_parameter<int>("max_cost", 100)))
{
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10);
}

void CostmapNode::lidarCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  auto grid = costmap_.generateCostmap(scan, this->now());
  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
