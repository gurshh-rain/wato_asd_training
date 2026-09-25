#include "costmap_core.hpp"

#include <cmath>
#include <algorithm>
#include <vector>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger,
                         double resolution,
                         double grid_size,
                         double inflation_radius,
                         int max_cost)
  : logger_(logger),
    resolution_(resolution),
    grid_size_(grid_size),
    inflation_radius_(inflation_radius),
    max_cost_(max_cost),
    width_(static_cast<unsigned int>(std::ceil(grid_size_ / resolution_))),
    height_(static_cast<unsigned int>(std::ceil(grid_size_ / resolution_)))
{
  RCLCPP_INFO(logger_, "CostmapCore initialized, grid: %ux%u", width_, height_);
}

nav_msgs::msg::OccupancyGrid CostmapCore::generateCostmap(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
    const rclcpp::Time& now) const
{
  nav_msgs::msg::OccupancyGrid grid;

  // Header
  grid.header.stamp = now;
  grid.header.frame_id = scan->header.frame_id;

  // Map metadata
  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width = width_;
  grid.info.height = height_;
  grid.info.map_load_time = now;
  grid.info.origin.position.x = -grid_size_ / 2.0;
  grid.info.origin.position.y = -grid_size_ / 2.0;
  grid.info.origin.position.z = 0.05;
  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Unknown by default
  grid.data.assign(width_ * height_, static_cast<int8_t>(-1));

  // Mark obstacle cells from the LaserScan
  std::vector<std::pair<int, int>> occupied_cells;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    // Skip invalid ranges
    if (range < scan->range_min || range > scan->range_max ||
        std::isnan(range) || std::isinf(range)) {
      continue;
    }

    double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Convert to grid indices
    int gx = static_cast<int>((x - grid.info.origin.position.x) / resolution_);
    int gy = static_cast<int>((y - grid.info.origin.position.y) / resolution_);

    if (gx >= 0 && gx < static_cast<int>(width_) &&
        gy >= 0 && gy < static_cast<int>(height_)) {
      int idx = gy * static_cast<int>(width_) + gx;
      grid.data[idx] = static_cast<int8_t>(100);
      occupied_cells.emplace_back(gx, gy);
    }
  }

  // Inflate obstacles out to the inflation radius
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  for (const auto& cell : occupied_cells) {
    int ox = cell.first;
    int oy = cell.second;

    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;

        if (nx < 0 || nx >= static_cast<int>(width_) ||
            ny < 0 || ny >= static_cast<int>(height_)) {
          continue;
        }

        double dist = std::sqrt(static_cast<double>(dx * dx) +
                                static_cast<double>(dy * dy)) * resolution_;
        if (dist > inflation_radius_) {
          continue;
        }

        int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
        int nidx = ny * static_cast<int>(width_) + nx;

        if (cost > grid.data[nidx]) {
          grid.data[nidx] = static_cast<int8_t>(cost);
        }
      }
    }
  }

  return grid;
}

}
