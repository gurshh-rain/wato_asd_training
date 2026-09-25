#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    robot::PlannerCore planner_;
    robot::PlannerCore fallback_planner_;
    State state_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fallback_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr dynamic_costmap_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    geometry_msgs::msg::PointStamped goal_;
    bool map_changed_;
    double goal_tolerance_;
    double progress_timeout_;
    double footprint_radius_;
    double start_clearance_radius_;
    double best_goal_distance_;
    rclcpp::Time last_progress_time_;
    void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void dynamicCostmapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void goalCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void timerCallback();
    void planPath();
    void applyFootprint(nav_msgs::msg::OccupancyGrid& map) const;
    void clearStartRegion(nav_msgs::msg::OccupancyGrid& map,
                          const nav_msgs::msg::OccupancyGrid& original) const;
    double goalDistance() const;
};

#endif
