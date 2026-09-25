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
  grid.header.stamp = scan->header.stamp;
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

  grid.data.assign(width_ * height_, static_cast<int8_t>(-1));
  std::vector<std::pair<int, int>> occupied_cells;
  int start_x = static_cast<int>((0.0 - grid.info.origin.position.x) / resolution_);
  int start_y = static_cast<int>((0.0 - grid.info.origin.position.y) / resolution_);
  auto clear_ray = [&](int end_x, int end_y) {
    int x = start_x;
    int y = start_y;
    int dx = std::abs(end_x - start_x);
    int sx = start_x < end_x ? 1 : -1;
    int dy = -std::abs(end_y - start_y);
    int sy = start_y < end_y ? 1 : -1;
    int error = dx + dy;
    while (true) {
      if (x >= 0 && x < static_cast<int>(width_) &&
          y >= 0 && y < static_cast<int>(height_)) {
        grid.data[y * static_cast<int>(width_) + x] = 0;
      }
      if (x == end_x && y == end_y) {
        break;
      }
      int twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y += sy;
      }
    }
  };
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double measured_range = scan->ranges[i];
    if (std::isnan(measured_range) || measured_range < scan->range_min) {
      continue;
    }
    bool hit = std::isfinite(measured_range) && measured_range < scan->range_max;
    double ray_range = hit ? measured_range : scan->range_max;
    double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    double x = ray_range * std::cos(angle);
    double y = ray_range * std::sin(angle);
    int gx = static_cast<int>((x - grid.info.origin.position.x) / resolution_);
    int gy = static_cast<int>((y - grid.info.origin.position.y) / resolution_);
    clear_ray(gx, gy);
    if (hit && gx >= 0 && gx < static_cast<int>(width_) &&
        gy >= 0 && gy < static_cast<int>(height_)) {
      occupied_cells.emplace_back(gx, gy);
    }
  }
  for (const auto& cell : occupied_cells) {
    grid.data[cell.second * static_cast<int>(width_) + cell.first] = 100;
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
