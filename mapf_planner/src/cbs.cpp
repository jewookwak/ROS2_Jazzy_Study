#include "mapf_planner/cbs.hpp"
#include <queue>
#include <unordered_set>
#include <set>
#include <cmath>

// ========== CBSNode 정의 ==========
struct CBSNode {
    std::vector<std::vector<Position>> paths;
    std::vector<Constraint> constraints;
    int cost;
    std::vector<Conflict> conflicts;
    int id;

    bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }
};

// ========== CBSSolver 생성자 구현 ==========
CBSSolver::CBSSolver(
    const std::vector<std::vector<bool>>& map,
    const std::vector<Position>& starts,
    const std::vector<Position>& goals
) : map_(map), starts_(starts), goals_(goals) {}

// ========== 충돌 탐지 ==========
std::vector<Conflict> CBSSolver::detectCollisions(const std::vector<std::vector<Position>>& paths) const {
    std::vector<Conflict> conflicts;
    size_t max_t = 0;
    for (const auto& p : paths) {
        if (p.size() > max_t) max_t = p.size();
    }
    for (size_t t = 0; t < max_t; ++t) {
        for (size_t i = 0; i < paths.size(); ++i) {
            Position pi = (t < paths[i].size()) ? paths[i][t] : paths[i].back();
            for (size_t j = i + 1; j < paths.size(); ++j) {
                Position pj = (t < paths[j].size()) ? paths[j][t] : paths[j].back();
                // Vertex conflict
                if (pi == pj) {
                    conflicts.push_back(Conflict{static_cast<int>(i), static_cast<int>(j), static_cast<int>(t), {pi}});
                }
                // Edge conflict
                if (t > 0) {
                    Position pi_prev = (t-1 < paths[i].size()) ? paths[i][t-1] : paths[i].back();
                    Position pj_prev = (t-1 < paths[j].size()) ? paths[j][t-1] : paths[j].back();
                    if (pi_prev == pj && pj_prev == pi) {
                        conflicts.push_back(Conflict{static_cast<int>(i), static_cast<int>(j), static_cast<int>(t),
                            {pi_prev, pi}});
                    }
                }
            }
        }
    }
    return conflicts;
}

// ========== 표준 Conflict 분할 ==========
std::vector<Constraint> CBSSolver::standardSplitting(const Conflict& conflict) const {
    std::vector<Constraint> constraints;
    // Vertex conflict: loc.size() == 1
    if (conflict.loc.size() == 1) {
        constraints.push_back(Constraint{conflict.agent1, conflict.timestep, conflict.loc});
        constraints.push_back(Constraint{conflict.agent2, conflict.timestep, conflict.loc});
    }
    // Edge conflict: loc.size() == 2
    else if (conflict.loc.size() == 2) {
        constraints.push_back(Constraint{conflict.agent1, conflict.timestep, {conflict.loc[0], conflict.loc[1]}});
        constraints.push_back(Constraint{conflict.agent2, conflict.timestep, {conflict.loc[1], conflict.loc[0]}});
    }
    return constraints;
}

// ========== Disjoint Splitting ==========
std::vector<Constraint> CBSSolver::disjointSplitting(const Conflict& conflict) const {
    // 간단하게 표준 분할과 동일하게 리턴 (실제 구현시 random이나 heuristic 선택 가능)
    return standardSplitting(conflict);
}

// ========== CBS 메인 알고리즘 ==========
std::vector<std::vector<Position>> CBSSolver::findSolution(bool disjoint) {
    AStarPlanner planner(map_);
    std::priority_queue<CBSNode, std::vector<CBSNode>, std::greater<>> open;
    CBSNode root;
    root.cost = 0;
    root.id = 0;

    for (size_t i = 0; i < starts_.size(); ++i) {
        std::vector<Constraint> empty;
        // === 맨해튼 휴리스틱 계산 ===
        std::vector<std::vector<int>> heuristics(map_.size(), std::vector<int>(map_[0].size(), 0));
        for (size_t y = 0; y < map_.size(); ++y) {
            for (size_t x = 0; x < map_[0].size(); ++x) {
                heuristics[y][x] = std::abs(goals_[i].first - static_cast<int>(y))
                                 + std::abs(goals_[i].second - static_cast<int>(x));
            }
        }
        auto path = planner.findPath(starts_[i], goals_[i], heuristics, i, empty);
        if (path.empty()) return {};
        root.paths.push_back(path);
        root.cost += path.size();
    }
    root.conflicts = detectCollisions(root.paths);
    open.push(root);

    int node_id = 1;

    while (!open.empty()) {
        CBSNode curr = open.top(); open.pop();
        if (curr.conflicts.empty()) return curr.paths;

        Conflict conflict = curr.conflicts[0];
        auto constraints = disjoint ? disjointSplitting(conflict) : standardSplitting(conflict);

        for (const auto& constr : constraints) {
            CBSNode child = curr;
            child.id = node_id++;
            child.constraints.push_back(constr);
            AStarPlanner planner(map_);

            // === 맨해튼 휴리스틱 계산 ===
            std::vector<std::vector<int>> heuristics(map_.size(), std::vector<int>(map_[0].size(), 0));
            for (size_t y = 0; y < map_.size(); ++y) {
                for (size_t x = 0; x < map_[0].size(); ++x) {
                    heuristics[y][x] = std::abs(goals_[constr.agent].first - static_cast<int>(y))
                                     + std::abs(goals_[constr.agent].second - static_cast<int>(x));
                }
            }

            auto path = planner.findPath(
                starts_[constr.agent],
                goals_[constr.agent],
                heuristics,
                constr.agent,
                child.constraints
            );
            if (path.empty()) continue;

            child.paths[constr.agent] = path;
            child.cost = 0;
            for (const auto& p : child.paths) child.cost += p.size();
            child.conflicts = detectCollisions(child.paths);
            open.push(child);
        }
    }
    return {};
}
