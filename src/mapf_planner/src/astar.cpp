#include "mapf_planner/astar.hpp"
#include <unordered_set>
#include <iostream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

AStarPlanner::AStarPlanner(const std::vector<std::vector<bool>>& map) : map_(map) {
    directions_ = {{0,1},{1,0},{0,-1},{-1,0},{0,0}};
}
// constraints는 agent가 해당위치에 특정시간에 가지 못하도록하는 제약조건이다.
bool AStarPlanner::isConstrained(const Position& curr, const Position& next, int timestep, int agent,
                                 const std::vector<Constraint>& constraints) const {
    for (const auto& c : constraints) {
        if (c.agent != agent) continue;
        if (c.timestep != timestep) continue;
        if (c.loc.size() == 1 && c.loc[0] == next)
            return true;
        if (c.loc.size() == 2 && c.loc[0] == curr && c.loc[1] == next)
            return true;
    }
    return false;
}

std::vector<Position> AStarPlanner::findPath(const Position& start,
                                             const Position& goal,
                                             const std::vector<std::vector<int>>& heuristics,
                                             int agent_id,
                                             const std::vector<Constraint>& constraints) {
    auto cmp = [](Node* a, Node* b) { return a->f_val() > b->f_val(); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_list(cmp);

    std::unordered_map<std::string, Node*> visited;

    auto to_key = [](const Position& p, int t) {
        return std::to_string(p.first) + "," + std::to_string(p.second) + "," + std::to_string(t);
    };

    Node* root = new Node{start, 0, heuristics[start.first][start.second], 0, nullptr};
    open_list.push(root);

    while (!open_list.empty()) {
        Node* curr = open_list.top(); open_list.pop();
        if (curr->pos == goal && !isConstrained(curr->pos, curr->pos, curr->timestep + 1, agent_id, constraints)) {
            std::vector<Position> path;
            while (curr) {
                path.push_back(curr->pos);
                curr = curr->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::string key = to_key(curr->pos, curr->timestep);
        if (visited.count(key)) continue;
        visited[key] = curr;

        for (const auto& dir : directions_) {
            Position next = {curr->pos.first + dir.first, curr->pos.second + dir.second};
            if (next.first < 0 || next.second < 0 ||
                static_cast<size_t>(next.first) >= map_.size() ||
                static_cast<size_t>(next.second) >= map_[0].size()) continue;
            if (map_[next.first][next.second]) continue;
            if (isConstrained(curr->pos, next, curr->timestep + 1, agent_id, constraints)) continue;

            int g = curr->g_val + 1;
            int h = heuristics[next.first][next.second];
            Node* child = new Node{next, g, h, curr->timestep + 1, curr};
            open_list.push(child);
        }
    }

    return {};
}
