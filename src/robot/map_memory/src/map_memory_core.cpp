#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger,
                             double resolution,
                             unsigned int width,
                             unsigned int height,
                             double origin_x,
                             double origin_y,
                             const std::string& frame_id,
                             double distance_threshold)
  : logger_(logger),
    resolution_(resolution),
    width_(width),
    height_(height),
    origin_x_(origin_x),
    origin_y_(origin_y),
    frame_id_(frame_id),
    distance_threshold_(distance_threshold),
    has_last_update_position_(false),
    should_update_(false),
    costmap_updated_(false)
{
  initializeMap();
}

void MapMemoryCore::initializeMap()
{
  global_map_.info.resolution = static_cast<float>(resolution_);
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin.position.x = origin_x_;
  global_map_.info.origin.position.y = origin_y_;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width_ * height_, 0);
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
{
  latest_costmap_ = msg;
  costmap_updated_ = true;
}

void MapMemoryCore::updateOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  latest_odom_ = msg;
  if (!has_last_update_position_) {
    last_update_position_ = msg->pose.pose.position;
    has_last_update_position_ = true;
    should_update_ = true;
  } else {
    double dist = euclideanDistance(msg->pose.pose.position, last_update_position_);
    if (dist >= distance_threshold_) {
      should_update_ = true;
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::generateMap(const rclcpp::Time& now)
{
  global_map_.header.stamp = now;
  global_map_.header.frame_id = frame_id_;
  if (should_update_ && costmap_updated_ && latest_costmap_ && latest_odom_) {
    double robot_x = latest_odom_->pose.pose.position.x;
    double robot_y = latest_odom_->pose.pose.position.y;
    double yaw = yawFromQuaternion(latest_odom_->pose.pose.orientation);
    unsigned int costmap_width = latest_costmap_->info.width;
    double c_res = latest_costmap_->info.resolution;
    double c_origin_x = latest_costmap_->info.origin.position.x;
    double c_origin_y = latest_costmap_->info.origin.position.y;
    for (unsigned int j = 0; j < latest_costmap_->info.height; ++j) {
      for (unsigned int i = 0; i < costmap_width; ++i) {
        int8_t value = latest_costmap_->data[j * costmap_width + i];
        if (value < 0) {
          continue;
        }
        double cx = c_origin_x + (static_cast<double>(i) + 0.5) * c_res;
        double cy = c_origin_y + (static_cast<double>(j) + 0.5) * c_res;
        double mx = robot_x + cx * std::cos(yaw) - cy * std::sin(yaw);
        double my = robot_y + cx * std::sin(yaw) + cy * std::cos(yaw);
        int base_mx = static_cast<int>((mx - origin_x_) / resolution_);
        int base_my = static_cast<int>((my - origin_y_) / resolution_);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int mx_i = base_mx + dx;
            int my_i = base_my + dy;
            if (mx_i < 0 || mx_i >= static_cast<int>(width_) ||
                my_i < 0 || my_i >= static_cast<int>(height_)) {
              continue;
            }
            global_map_.data[static_cast<unsigned int>(my_i) * width_ +
                             static_cast<unsigned int>(mx_i)] = value;
          }
        }
      }
    }
    last_update_position_ = latest_odom_->pose.pose.position;
    should_update_ = false;
    costmap_updated_ = false;
  }
  return global_map_;
}

double MapMemoryCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double MapMemoryCore::euclideanDistance(const geometry_msgs::msg::Point& a,
                                        const geometry_msgs::msg::Point& b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}
