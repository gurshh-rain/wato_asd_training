#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace robot
{

namespace
{

constexpr double kEpsilon = 1e-9;

bool lessKey(const std::pair<double, double>& a,
             const std::pair<double, double>& b)
{
  return a.first < b.first - kEpsilon ||
         (std::abs(a.first - b.first) <= kEpsilon &&
          a.second < b.second - kEpsilon);
}

}

PlannerCore::PlannerCore(const rclcpp::Logger& logger,
                         int occupied_threshold,
                         bool allow_diagonal)
  : logger_(logger),
    occupied_threshold_(occupied_threshold),
    allow_diagonal_(allow_diagonal),
    initialized_(false),
    start_index_(-1),
    last_start_index_(-1),
    goal_index_(-1),
    key_modifier_(0.0)
{
}

nav_msgs::msg::Path PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map,
                                      const geometry_msgs::msg::Point& start,
                                      const geometry_msgs::msg::Point& goal,
                                      const rclcpp::Time& stamp)
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = map.header.frame_id;
  CellIndex start_cell;
  CellIndex goal_cell;
  if (map.data.empty() || !worldToGrid(map, start, start_cell) ||
      !worldToGrid(map, goal, goal_cell)) {
    return path;
  }
  int width = static_cast<int>(map.info.width);
  int new_start = start_cell.y * width + start_cell.x;
  int new_goal = goal_cell.y * width + goal_cell.x;
  if (!initialized_ || !sameGeometry(map) || new_goal != goal_index_) {
    initialize(map, new_start, new_goal);
  } else {
    if (new_start != start_index_) {
      key_modifier_ += heuristic(last_start_index_, new_start);
      start_index_ = new_start;
      last_start_index_ = new_start;
    }
    applyMapChanges(map);
  }
  if (!traversable(start_index_) || !traversable(goal_index_)) {
    return path;
  }
  computeShortestPath();
  if (!std::isfinite(g_[start_index_])) {
    return path;
  }
  std::vector<int> indices;
  indices.push_back(start_index_);
  std::unordered_set<int> visited;
  visited.insert(start_index_);
  int current = start_index_;
  int maximum_steps = static_cast<int>(map_.data.size());
  while (current != goal_index_ && static_cast<int>(indices.size()) < maximum_steps) {
    int best_neighbor = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int neighbor : neighbors(current)) {
      double edge = edgeCost(current, neighbor);
      if (!std::isfinite(edge) || !std::isfinite(g_[neighbor])) {
        continue;
      }
      double candidate = edge + g_[neighbor];
      if (candidate < best_cost - kEpsilon) {
        best_cost = candidate;
        best_neighbor = neighbor;
      }
    }
    if (best_neighbor < 0 || visited.count(best_neighbor) > 0) {
      return nav_msgs::msg::Path();
    }
    current = best_neighbor;
    visited.insert(current);
    indices.push_back(current);
  }
  if (indices.back() != goal_index_) {
    return nav_msgs::msg::Path();
  }
  path.poses.reserve(indices.size());
  for (int index : indices) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = gridToWorld(
        map_, {index % static_cast<int>(map_.info.width),
               index / static_cast<int>(map_.info.width)});
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path.poses.back().pose.position = goal;
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

bool PlannerCore::sameGeometry(const nav_msgs::msg::OccupancyGrid& map) const
{
  return map.info.width == map_.info.width &&
         map.info.height == map_.info.height &&
         std::abs(map.info.resolution - map_.info.resolution) <= kEpsilon &&
         std::abs(map.info.origin.position.x - map_.info.origin.position.x) <= kEpsilon &&
         std::abs(map.info.origin.position.y - map_.info.origin.position.y) <= kEpsilon;
}

bool PlannerCore::traversable(int index) const
{
  if (index < 0 || index >= static_cast<int>(map_.data.size())) {
    return false;
  }
  int value = map_.data[static_cast<std::size_t>(index)];
  return value >= 0 && value < occupied_threshold_;
}

bool PlannerCore::traversable(int x, int y) const
{
  if (x < 0 || y < 0 || x >= static_cast<int>(map_.info.width) ||
      y >= static_cast<int>(map_.info.height)) {
    return false;
  }
  return traversable(y * static_cast<int>(map_.info.width) + x);
}

std::vector<int> PlannerCore::neighbors(int index) const
{
  std::vector<int> result;
  int width = static_cast<int>(map_.info.width);
  int x = index % width;
  int y = index / width;
  const int directions[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };
  int count = allow_diagonal_ ? 8 : 4;
  for (int i = 0; i < count; ++i) {
    int nx = x + directions[i][0];
    int ny = y + directions[i][1];
    if (nx >= 0 && ny >= 0 && nx < width && ny < static_cast<int>(map_.info.height)) {
      result.push_back(ny * width + nx);
    }
  }
  return result;
}

double PlannerCore::edgeCost(int from, int to) const
{
  if (!traversable(from) || !traversable(to)) {
    return std::numeric_limits<double>::infinity();
  }
  int width = static_cast<int>(map_.info.width);
  int from_x = from % width;
  int from_y = from / width;
  int to_x = to % width;
  int to_y = to / width;
  int dx = to_x - from_x;
  int dy = to_y - from_y;
  if (dx != 0 && dy != 0 &&
      (!traversable(from_x + dx, from_y) ||
       !traversable(from_x, from_y + dy))) {
    return std::numeric_limits<double>::infinity();
  }
  double step = dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0;
  double cost = std::max(0, static_cast<int>(map_.data[static_cast<std::size_t>(to)])) / 100.0;
  return step * (1.0 + cost);
}

double PlannerCore::heuristic(int from, int to) const
{
  int width = static_cast<int>(map_.info.width);
  int dx = std::abs(from % width - to % width);
  int dy = std::abs(from / width - to / width);
  if (!allow_diagonal_) {
    return static_cast<double>(dx + dy);
  }
  int diagonal = std::min(dx, dy);
  int straight = std::max(dx, dy) - diagonal;
  return std::sqrt(2.0) * static_cast<double>(diagonal) +
         static_cast<double>(straight);
}

std::pair<double, double> PlannerCore::calculateKey(int index) const
{
  double minimum = std::min(g_[index], rhs_[index]);
  return {minimum + heuristic(start_index_, index) + key_modifier_, minimum};
}

bool PlannerCore::inconsistent(int index) const
{
  if (std::isfinite(g_[index]) != std::isfinite(rhs_[index])) {
    return true;
  }
  if (!std::isfinite(g_[index]) && !std::isfinite(rhs_[index])) {
    return false;
  }
  return std::abs(g_[index] - rhs_[index]) > kEpsilon;
}

void PlannerCore::initialize(const nav_msgs::msg::OccupancyGrid& map,
                             int start_index, int goal_index)
{
  map_ = map;
  start_index_ = start_index;
  last_start_index_ = start_index;
  goal_index_ = goal_index;
  key_modifier_ = 0.0;
  double infinity = std::numeric_limits<double>::infinity();
  g_.assign(map_.data.size(), infinity);
  rhs_.assign(map_.data.size(), infinity);
  queue_versions_.assign(map_.data.size(), 0);
  open_ = decltype(open_)();
  rhs_[goal_index_] = traversable(goal_index_) ? 0.0 : infinity;
  enqueue(goal_index_);
  initialized_ = true;
}

void PlannerCore::applyMapChanges(const nav_msgs::msg::OccupancyGrid& map)
{
  std::vector<int> changed;
  for (std::size_t i = 0; i < map.data.size(); ++i) {
    if (map.data[i] != map_.data[i]) {
      changed.push_back(static_cast<int>(i));
    }
  }
  map_ = map;
  int width = static_cast<int>(map_.info.width);
  int height = static_cast<int>(map_.info.height);
  std::unordered_set<int> affected;
  for (int index : changed) {
    int x = index % width;
    int y = index / width;
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        int nx = x + dx;
        int ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
          affected.insert(ny * width + nx);
        }
      }
    }
  }
  for (int index : affected) {
    updateVertex(index);
  }
}

void PlannerCore::updateVertex(int index)
{
  double infinity = std::numeric_limits<double>::infinity();
  if (index == goal_index_) {
    rhs_[index] = traversable(index) ? 0.0 : infinity;
  } else if (!traversable(index)) {
    rhs_[index] = infinity;
  } else {
    double best = infinity;
    for (int successor : neighbors(index)) {
      double edge = edgeCost(index, successor);
      if (std::isfinite(edge) && std::isfinite(g_[successor])) {
        best = std::min(best, edge + g_[successor]);
      }
    }
    rhs_[index] = best;
  }
  enqueue(index);
}

void PlannerCore::enqueue(int index)
{
  queue_versions_[index]++;
  if (!inconsistent(index)) {
    return;
  }
  auto key = calculateKey(index);
  open_.push({index, key.first, key.second, queue_versions_[index]});
}

bool PlannerCore::topValid(DStarQueueNode& node)
{
  while (!open_.empty()) {
    node = open_.top();
    if (node.version != queue_versions_[node.index] || !inconsistent(node.index)) {
      open_.pop();
      continue;
    }
    return true;
  }
  return false;
}

bool PlannerCore::popValid(DStarQueueNode& node)
{
  if (!topValid(node)) {
    return false;
  }
  open_.pop();
  return true;
}

void PlannerCore::computeShortestPath()
{
  int maximum_expansions = static_cast<int>(map_.data.size()) * 20;
  int expansions = 0;
  while (expansions < maximum_expansions) {
    DStarQueueNode top;
    if (!topValid(top)) {
      break;
    }
    auto top_key = std::make_pair(top.first, top.second);
    auto start_key = calculateKey(start_index_);
    if (!lessKey(top_key, start_key) && !inconsistent(start_index_)) {
      break;
    }
    popValid(top);
    auto old_key = std::make_pair(top.first, top.second);
    auto new_key = calculateKey(top.index);
    if (lessKey(old_key, new_key)) {
      enqueue(top.index);
    } else if (g_[top.index] > rhs_[top.index]) {
      g_[top.index] = rhs_[top.index];
      queue_versions_[top.index]++;
      for (int predecessor : neighbors(top.index)) {
        updateVertex(predecessor);
      }
    } else {
      g_[top.index] = std::numeric_limits<double>::infinity();
      updateVertex(top.index);
      for (int predecessor : neighbors(top.index)) {
        updateVertex(predecessor);
      }
    }
    expansions++;
  }
}

}
