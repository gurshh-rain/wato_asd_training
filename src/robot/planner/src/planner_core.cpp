#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace robot
{

namespace
{

struct QueueNode {
  int index;
  double f_score;
};

struct CompareScore {
  bool operator()(const QueueNode& a, const QueueNode& b) const
  {
    return a.f_score > b.f_score;
  }
};

}

PlannerCore::PlannerCore(const rclcpp::Logger& logger,
                         int occupied_threshold,
                         bool allow_diagonal)
  : logger_(logger),
    occupied_threshold_(occupied_threshold),
    allow_diagonal_(allow_diagonal)
{
}

nav_msgs::msg::Path PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map,
                                      const geometry_msgs::msg::Point& start,
                                      const geometry_msgs::msg::Point& goal,
                                      const rclcpp::Time& stamp) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = map.header.frame_id;
  CellIndex start_cell;
  CellIndex goal_cell;
  if (map.data.empty() || !worldToGrid(map, start, start_cell) ||
      !worldToGrid(map, goal, goal_cell) ||
      !traversable(map, start_cell.x, start_cell.y) ||
      !traversable(map, goal_cell.x, goal_cell.y)) {
    return path;
  }
  int width = static_cast<int>(map.info.width);
  int height = static_cast<int>(map.info.height);
  int cell_count = width * height;
  int start_index = start_cell.y * width + start_cell.x;
  int goal_index = goal_cell.y * width + goal_cell.x;
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(cell_count, -1);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, CompareScore> open;
  auto heuristic = [&goal_cell](int x, int y) {
    return std::hypot(static_cast<double>(goal_cell.x - x),
                      static_cast<double>(goal_cell.y - y));
  };
  g_score[start_index] = 0.0;
  open.push({start_index, heuristic(start_cell.x, start_cell.y)});
  const int directions[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };
  bool found = false;
  while (!open.empty()) {
    int current = open.top().index;
    open.pop();
    if (closed[current]) {
      continue;
    }
    if (current == goal_index) {
      found = true;
      break;
    }
    closed[current] = true;
    int cx = current % width;
    int cy = current / width;
    int direction_count = allow_diagonal_ ? 8 : 4;
    for (int i = 0; i < direction_count; ++i) {
      int dx = directions[i][0];
      int dy = directions[i][1];
      int nx = cx + dx;
      int ny = cy + dy;
      if (!traversable(map, nx, ny)) {
        continue;
      }
      if (dx != 0 && dy != 0 &&
          (!traversable(map, cx + dx, cy) || !traversable(map, cx, cy + dy))) {
        continue;
      }
      int neighbor = ny * width + nx;
      if (closed[neighbor]) {
        continue;
      }
      double step_cost = dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0;
      double map_cost = std::max(0, static_cast<int>(map.data[neighbor])) / 100.0;
      double tentative = g_score[current] + step_cost * (1.0 + map_cost);
      if (tentative >= g_score[neighbor]) {
        continue;
      }
      came_from[neighbor] = current;
      g_score[neighbor] = tentative;
      open.push({neighbor, tentative + heuristic(nx, ny)});
    }
  }
  if (!found) {
    return path;
  }
  std::vector<CellIndex> cells;
  for (int current = goal_index; current != -1; current = came_from[current]) {
    cells.push_back({current % width, current / width});
    if (current == start_index) {
      break;
    }
  }
  if (cells.empty() || !(cells.back() == start_cell)) {
    return path;
  }
  std::reverse(cells.begin(), cells.end());
  path.poses.reserve(cells.size());
  for (const auto& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = gridToWorld(map, cell);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  if (!path.poses.empty()) {
    path.poses.back().pose.position = goal;
  }
  return path;
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map,
                              const geometry_msgs::msg::Point& point,
                              CellIndex& cell) const
{
  double resolution = map.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }
  cell.x = static_cast<int>(std::floor((point.x - map.info.origin.position.x) / resolution));
  cell.y = static_cast<int>(std::floor((point.y - map.info.origin.position.y) / resolution));
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

geometry_msgs::msg::Point PlannerCore::gridToWorld(
    const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const
{
  geometry_msgs::msg::Point point;
  point.x = map.info.origin.position.x +
            (static_cast<double>(cell.x) + 0.5) * map.info.resolution;
  point.y = map.info.origin.position.y +
            (static_cast<double>(cell.y) + 0.5) * map.info.resolution;
  return point;
}

bool PlannerCore::traversable(const nav_msgs::msg::OccupancyGrid& map, int x, int y) const
{
  if (x < 0 || x >= static_cast<int>(map.info.width) ||
      y < 0 || y >= static_cast<int>(map.info.height)) {
    return false;
  }
  int value = map.data[static_cast<std::size_t>(y) * map.info.width +
                       static_cast<std::size_t>(x)];
  return value >= 0 && value < occupied_threshold_;
}

}
