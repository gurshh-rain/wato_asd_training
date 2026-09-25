#include "map_memory_core.hpp"
#include "gtest/gtest.h"
#include <algorithm>

TEST(MapMemory, ClearsObservedObstacleWhileRobotIsStationary)
{
  robot::MapMemoryCore core(rclcpp::get_logger("test"), .5, 8, 8, -2., -2., "sim_world", .25);
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->pose.pose.orientation.w = 1.0;
  core.updateOdom(odom);
  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid->info.width = 4;
  grid->info.height = 4;
  grid->info.resolution = 1.;
  grid->info.origin.position.x = -2.;
  grid->info.origin.position.y = -2.;
  grid->info.origin.orientation.w = 1.;
  grid->data.assign(16, 0);
  grid->data[10] = 100;
  core.updateCostmap(grid);
  auto first = core.generateMap(rclcpp::Time(0));
  EXPECT_EQ(std::count(first.data.begin(), first.data.end(), 100), 4);
  grid->data[10] = 0;
  core.updateCostmap(grid);
  auto second = core.generateMap(rclcpp::Time(1));
  EXPECT_EQ(std::count(second.data.begin(), second.data.end(), 100), 0);
}
