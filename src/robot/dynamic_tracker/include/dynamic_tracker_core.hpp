#ifndef DYNAMIC_TRACKER_CORE_HPP_
#define DYNAMIC_TRACKER_CORE_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace robot
{

struct Detection {
  double x;
  double y;
  double radius;
};

struct AxisFilter {
  double position;
  double velocity;
  double p00;
  double p01;
  double p11;
};

struct ObstacleTrack {
  int id;
  AxisFilter x;
  AxisFilter y;
  double radius;
  int hits;
  int misses;
  int moving_hits;
  rclcpp::Time stamp;
  rclcpp::Time dynamic_until;
};

struct TrackerOutput {
  nav_msgs::msg::OccupancyGrid dynamic_costmap;
  visualization_msgs::msg::MarkerArray markers;
};

class DynamicTrackerCore {
  public:
    DynamicTrackerCore(const rclcpp::Logger& logger,
                       double cluster_threshold,
                       int minimum_cluster_size,
                       double maximum_cluster_radius,
                       double association_gate,
                       double dynamic_speed_threshold,
                       double process_noise,
                       double measurement_noise,
                       double prediction_horizon,
                       double occupancy_horizon,
                       double prediction_step,
                       double robot_radius,
                       double safety_margin,
                       double motion_min_x,
                       double motion_max_x);
    TrackerOutput process(const sensor_msgs::msg::LaserScan& scan,
                          const nav_msgs::msg::Odometry& odom,
                          const nav_msgs::msg::OccupancyGrid& map,
                          const rclcpp::Time& stamp);

  private:
    rclcpp::Logger logger_;
    double cluster_threshold_;
    int minimum_cluster_size_;
    double maximum_cluster_radius_;
    double association_gate_;
    double dynamic_speed_threshold_;
    double process_noise_;
    double measurement_noise_;
    double prediction_horizon_;
    double occupancy_horizon_;
    double prediction_step_;
    double robot_radius_;
    double safety_margin_;
    double motion_min_x_;
    double motion_max_x_;
    int next_id_;
    std::vector<ObstacleTrack> tracks_;
    std::vector<Detection> detect(const sensor_msgs::msg::LaserScan& scan,
                                  const nav_msgs::msg::Odometry& odom) const;
    void predict(ObstacleTrack& track, double dt) const;
    void update(ObstacleTrack& track, const Detection& detection) const;
    void updateAxis(AxisFilter& axis, double measurement) const;
    bool isDynamic(const ObstacleTrack& track, const rclcpp::Time& now) const;
    geometry_msgs::msg::Point predictPosition(const ObstacleTrack& track,
                                               double time) const;
    void rasterize(nav_msgs::msg::OccupancyGrid& grid, double x, double y,
                   double radius, int8_t cost) const;
    visualization_msgs::msg::MarkerArray makeMarkers(const rclcpp::Time& stamp) const;
};

}

#endif
