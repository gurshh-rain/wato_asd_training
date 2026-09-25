#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    MapMemoryCore(const rclcpp::Logger& logger,
                  double resolution,
                  unsigned int width,
                  unsigned int height,
                  double origin_x,
                  double origin_y,
                  const std::string& frame_id,
                  double distance_threshold);
    void updateCostmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);
    void updateOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    nav_msgs::msg::OccupancyGrid generateMap(const rclcpp::Time& now);

  private:
    rclcpp::Logger logger_;
    double resolution_;
    unsigned int width_;
    unsigned int height_;
    double origin_x_;
    double origin_y_;
    std::string frame_id_;
    double distance_threshold_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
    geometry_msgs::msg::Point last_update_position_;
    bool has_last_update_position_;
    bool should_update_;
    bool costmap_updated_;
    nav_msgs::msg::OccupancyGrid global_map_;
    void initializeMap();
    double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const;
    double euclideanDistance(const geometry_msgs::msg::Point& a,
                             const geometry_msgs::msg::Point& b) const;
};

}

#endif
