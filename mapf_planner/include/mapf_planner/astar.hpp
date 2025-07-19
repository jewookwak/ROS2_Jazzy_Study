#pragma once

#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

using Position = std::pair<int, int>;

struct Constraint {
    int agent;
    int timestep;
    std::vector<Position> loc;
};

struct Node {
    Position pos;
    int g_val;
    int h_val;
    int timestep;
    Node* parent;

    int f_val() const { return g_val + h_val; }
};

class AStarPlanner {
public:
    AStarPlanner(const std::vector<std::vector<bool>>& map);

    std::vector<Position> findPath(const Position& start,
                                   const Position& goal,
                                   const std::vector<std::vector<int>>& heuristics,
                                   int agent_id,
                                   const std::vector<Constraint>& constraints);


private:
    std::vector<std::vector<bool>> map_;
    std::vector<Position> directions_;

    bool isConstrained(const Position& curr, const Position& next, int timestep, int agent,
                       const std::vector<Constraint>& constraints) const;
};
