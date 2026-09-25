#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger,
                         double resolution,
                         double grid_size,
                         double inflation_radius,
                         int max_cost);

    nav_msgs::msg::OccupancyGrid generateCostmap(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
        const rclcpp::Time& now) const;

  private:
    rclcpp::Logger logger_;
    double resolution_;
    double grid_size_;
    double inflation_radius_;
    int max_cost_;
    unsigned int width_;
    unsigned int height_;
};

}  

#endif
