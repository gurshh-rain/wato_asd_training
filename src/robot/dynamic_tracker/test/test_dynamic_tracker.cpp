#include <algorithm>
#include <cmath>
#include <limits>

#include "dynamic_tracker_core.hpp"
#include "gtest/gtest.h"

TEST(DynamicTrackerCore, PredictsMovingCluster)
{
  robot::DynamicTrackerCore tracker(
      rclcpp::get_logger("test"), 0.6, 2, 0.9, 1.2, 0.3,
      1.0, 0.15, 4.0, 1.5, 0.25, 0.8, 0.3, -12.0, 12.0);
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "sim_world";
  map.info.resolution = 0.1;
  map.info.width = 300;
  map.info.height = 300;
  map.info.origin.position.x = -15.0;
  map.info.origin.position.y = -15.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -0.1;
  scan.angle_increment = 0.01;
  scan.range_min = 0.08;
  scan.range_max = 20.0;
  scan.ranges.assign(21, std::numeric_limits<float>::infinity());
  robot::TrackerOutput output;
  for (int step = 0; step < 12; ++step) {
    float range = 5.0f + static_cast<float>(step) * 0.08f;
    scan.ranges[9] = range;
    scan.ranges[10] = range;
    scan.ranges[11] = range;
    output = tracker.process(scan, odom, map,
                             rclcpp::Time(static_cast<int64_t>(step) * 100000000LL));
  }
  EXPECT_GT(std::count_if(output.dynamic_costmap.data.begin(),
                          output.dynamic_costmap.data.end(),
                          [](int8_t value) { return value > 0; }), 0);
  EXPECT_GT(output.markers.markers.size(), 1u);
}
