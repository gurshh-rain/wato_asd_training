#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

struct CellIndex {
  int x;
  int y;

  bool operator==(const CellIndex& other) const
  {
    return x == other.x && y == other.y;
  }
};

class PlannerCore {
  public:
    PlannerCore(const rclcpp::Logger& logger, int occupied_threshold, bool allow_diagonal);
    nav_msgs::msg::Path plan(const nav_msgs::msg::OccupancyGrid& map,
                             const geometry_msgs::msg::Point& start,
                             const geometry_msgs::msg::Point& goal,
                             const rclcpp::Time& stamp) const;

  private:
    rclcpp::Logger logger_;
    int occupied_threshold_;
    bool allow_diagonal_;
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map,
                     const geometry_msgs::msg::Point& point,
                     CellIndex& cell) const;
    geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid& map,
                                          const CellIndex& cell) const;
    bool traversable(const nav_msgs::msg::OccupancyGrid& map, int x, int y) const;
};

}

#endif
