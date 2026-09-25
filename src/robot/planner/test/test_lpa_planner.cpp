#include <algorithm>

#include "gtest/gtest.h"
#include "planner_core.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeMap()
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "sim_world";
  map.info.resolution = 1.0;
  map.info.width = 20;
  map.info.height = 10;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}

void setWall(nav_msgs::msg::OccupancyGrid& map, int gap)
{
  for (int y = 0; y < static_cast<int>(map.info.height); ++y) {
    map.data[static_cast<std::size_t>(y) * map.info.width + 10] = y == gap ? 0 : 100;
  }
}

}

TEST(DStarLitePlanner, IncrementallyRepairsPathWhenCorridorChanges)
{
  robot::PlannerCore planner(rclcpp::get_logger("test"), 1, true);
  auto map = makeMap();
  geometry_msgs::msg::Point start;
  start.x = 1.5;
  start.y = 5.5;
  geometry_msgs::msg::Point goal;
  goal.x = 18.5;
  goal.y = 5.5;
  auto direct = planner.plan(map, start, goal, rclcpp::Time(0));
  ASSERT_FALSE(direct.poses.empty());

  setWall(map, 8);
  auto upper = planner.plan(map, start, goal, rclcpp::Time(1));
  ASSERT_FALSE(upper.poses.empty());
  EXPECT_TRUE(std::any_of(upper.poses.begin(), upper.poses.end(),
                          [](const auto& pose) {
                            return pose.pose.position.y >= 8.5;
                          }));

  setWall(map, 2);
  auto lower = planner.plan(map, start, goal, rclcpp::Time(2));
  ASSERT_FALSE(lower.poses.empty());
  EXPECT_TRUE(std::any_of(lower.poses.begin(), lower.poses.end(),
                          [](const auto& pose) {
                            return pose.pose.position.y <= 2.5;
                          }));

  setWall(map, -1);
  auto blocked = planner.plan(map, start, goal, rclcpp::Time(3));
  EXPECT_TRUE(blocked.poses.empty());
}
