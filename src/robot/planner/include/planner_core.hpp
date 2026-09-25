#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

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

struct DStarQueueNode {
  int index;
  double first;
  double second;
  std::uint64_t version;
};

struct CompareDStarKey {
  bool operator()(const DStarQueueNode& a, const DStarQueueNode& b) const
  {
    if (a.first != b.first) {
      return a.first > b.first;
    }
    if (a.second != b.second) {
      return a.second > b.second;
    }
    return a.version > b.version;
  }
};

class PlannerCore {
  public:
    PlannerCore(const rclcpp::Logger& logger, int occupied_threshold, bool allow_diagonal);
    nav_msgs::msg::Path plan(const nav_msgs::msg::OccupancyGrid& map,
                             const geometry_msgs::msg::Point& start,
                             const geometry_msgs::msg::Point& goal,
                             const rclcpp::Time& stamp);

  private:
    rclcpp::Logger logger_;
    int occupied_threshold_;
    bool allow_diagonal_;
    bool initialized_;
    int start_index_;
    int last_start_index_;
    int goal_index_;
    double key_modifier_;
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<double> g_;
    std::vector<double> rhs_;
    std::vector<std::uint64_t> queue_versions_;
    std::priority_queue<DStarQueueNode, std::vector<DStarQueueNode>, CompareDStarKey> open_;
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map,
                     const geometry_msgs::msg::Point& point,
                     CellIndex& cell) const;
    geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid& map,
                                          const CellIndex& cell) const;
    bool sameGeometry(const nav_msgs::msg::OccupancyGrid& map) const;
    bool traversable(int index) const;
    bool traversable(int x, int y) const;
    std::vector<int> neighbors(int index) const;
    double edgeCost(int from, int to) const;
    double heuristic(int from, int to) const;
    std::pair<double, double> calculateKey(int index) const;
    bool inconsistent(int index) const;
    void initialize(const nav_msgs::msg::OccupancyGrid& map,
                    int start_index, int goal_index);
    void applyMapChanges(const nav_msgs::msg::OccupancyGrid& map);
    void updateVertex(int index);
    void enqueue(int index);
    bool topValid(DStarQueueNode& node);
    bool popValid(DStarQueueNode& node);
    void computeShortestPath();
};

}

#endif
