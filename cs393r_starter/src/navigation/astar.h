#ifndef ASTAR_H
#define ASTAR_H

#include <memory>
#include <set>
#include <unordered_map>

#include "eigen3/Eigen/Dense"
#include "occupancy_map.h"

namespace navigation {

struct Node {
  Eigen::Vector2i position;
  std::shared_ptr<Node> parent;
  float gCost;                                   // Cost from start to the current node
  float hCost;                                   // Heuristic cost estimate to goal
  float fCost() const { return gCost + hCost; }  // Total cost

  bool operator==(const Node& other) const { return position == other.position; }

  // For std::set sorting
  bool operator<(const Node& other) const {
    return fCost() < other.fCost() || (fCost() == other.fCost() && hCost < other.hCost);
  }
};

struct Vector2iHash {
  std::size_t operator()(const Eigen::Vector2i& v) const {
    return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y());
  }
};

class AStar {
 public:
  AStar(const OccupancyMap& map) : map_(map) {}

  std::vector<Eigen::Vector2f> findPath(const Eigen::Vector2f& start,
                                        const Eigen::Vector2f& goal) {
    start_ = start;
    goal_ = goal;
    path_.clear();
    allNodes_.clear();

    if (FLAGS_v > 1) {
      std::cout << "======= [Astar] findPath =======" << std::endl;
      std::cout << "Start: " << start.transpose() << std::endl;
      std::cout << "Goal: " << goal.transpose() << std::endl;
    }

    auto [startXIdx, startYIdx] = map_.getIdx(start.x(), start.y());
    auto [goalXIdx, goalYIdx] = map_.getIdx(goal.x(), goal.y());
    Eigen::Vector2i startIdx(startXIdx, startYIdx);
    Eigen::Vector2i goalIdx(goalXIdx, goalYIdx);

    if (map_.isOccupied(startIdx) || map_.isOccupied(goalIdx)) {
      std::cout << "[Astar] Start or goal is occupied!" << std::endl;
      return {};
    }

    std::set<Node> openSet;

    Node startNode{startIdx, nullptr, 0.0f, HeuristicCost(startIdx, goalIdx)};
    openSet.insert(startNode);
    allNodes_[startIdx] = startNode;

    size_t num_expanded = 0;
    while (!openSet.empty()) {
      Node current = *openSet.begin();
      openSet.erase(openSet.begin());

      if (current.position == goalIdx) {
        // Saves last found path
        path_ = ReconstructPath(current);
        return path_;
      }

      for (const auto& dir : directions) {
        Eigen::Vector2i neighborPos = current.position + dir;
        if (map_.isOccupied(neighborPos)) continue;

        float tentativeGCost =
            current.gCost + (dir.array().abs().sum() == 2 ? M_SQRT2 : 1.0);
        if (allNodes_.find(neighborPos) == allNodes_.end() ||
            tentativeGCost < allNodes_[neighborPos].gCost) {
          Node neighbor{neighborPos,
                        std::make_shared<Node>(current),
                        tentativeGCost,
                        HeuristicCost(neighborPos, goalIdx)};
          openSet.insert(neighbor);
          allNodes_[neighborPos] = neighbor;
          num_expanded += 1;
        }
      }
    }
    std::cout << "Num nodes expanded before termination " << num_expanded << std::endl;
    std::cout << "[Astar] No path found!" << std::endl;

    return {};  // Return empty path if no path found
  }

  Eigen::Vector2f getPose(const Eigen::Vector2f& loc) {
    auto [xIdx, yIdx] = map_.getIdx(loc.x(), loc.y());
    return map_.getPose(xIdx, yIdx);
  }

  void visualization(VisualizationMsg& global_msg) {
    uint32_t path_color = 62762;
    visualization::DrawCross(start_, 0.5, 62762, global_msg);
    visualization::DrawCross(goal_, 0.5, 62762, global_msg);

    std::cout << "number of all nodes: " << allNodes_.size() << std::endl;

    for (const auto& [pos, node] : allNodes_) {
      const auto& pose = map_.getPose(node.position.x(), node.position.y());
      visualization::DrawPoint(pose, 62762, global_msg);
    }

    std::cout << "number of path nodes: " << path_.size() << std::endl;
    for (size_t i = 1; i < path_.size(); ++i) {
      visualization::DrawLine(path_[i - 1], path_[i], path_color, global_msg);
    }
  }

 private:
  const OccupancyMap& map_;
  std::vector<Eigen::Vector2i> directions{
      {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
  std::unordered_map<Eigen::Vector2i, Node, Vector2iHash> allNodes_;

  std::vector<Eigen::Vector2f> path_;
  Eigen::Vector2f start_;
  Eigen::Vector2f goal_;

  float HeuristicCost(const Eigen::Vector2i& a, const Eigen::Vector2i& b) const {
    return (a - b).cast<float>().norm();  // Euclidean distance as heuristic
  }

  std::vector<Eigen::Vector2f> ReconstructPath(const Node& goalNode) const {
    std::vector<Eigen::Vector2f> path;
    std::shared_ptr<Node> current = std::make_shared<Node>(goalNode);
    while (current != nullptr) {
      path.push_back(map_.getPose(current->position.x(), current->position.y()));
      current = current->parent;
    }
    std::reverse(path.begin(), path.end());

    if (FLAGS_v > 1) {
      std::cout << "Path: ";
      for (const auto& p : path_) {
        std::cout << p.transpose() << " ";
      }
      std::cout << std::endl;
    }

    return path;
  }
};

}  // namespace navigation

#endif  // ASTAR_H