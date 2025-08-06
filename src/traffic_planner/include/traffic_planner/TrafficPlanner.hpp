#pragma once

#include <vector>
#include <utility>
#include <queue>
#include <unordered_map>
#include <string>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <set>
#include <cmath>

using Position = std::pair<int, int>;

struct Constraint {
    int agent;
    int timestep;
    std::vector<Position> loc;  // size=1이면 vertex constraint, size=2이면 edge constraint
};

struct Conflict {
    int agent1, agent2;
    int timestep;
    std::vector<Position> loc;  // 충돌 위치 (vertex 또는 edge)
};

struct Node {
    Position pos;
    int g_val;
    int h_val;
    int timestep;
    Node* parent;

    int f_val() const { return g_val + h_val; }
};

// CBS 노드 구조체
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

class AStarPlanner {
public:
    AStarPlanner(const std::vector<std::vector<bool>>& map);

    std::vector<Position> findPath(
        const Position& start,
        const Position& goal,
        const std::vector<std::vector<int>>& heuristics,
        int agent_id,
        const std::vector<Constraint>& constraints
    );

private:
    std::vector<std::vector<bool>> map_;
    std::vector<Position> directions_;

    bool isConstrained(
        const Position& curr,
        const Position& next,
        int timestep,
        int agent,
        const std::vector<Constraint>& constraints
    ) const;
};

class TrafficPlanner {
public:
    TrafficPlanner(const std::vector<std::vector<bool>>& map);
    
    // 메인 경로 계획 함수
    std::vector<std::vector<Position>> planPaths(
        const std::vector<Position>& starts,
        const std::vector<Position>& goals,
        bool use_disjoint_splitting = true,
        const std::vector<Constraint>& initial_constraints = {}
    );

    // 맵 업데이트
    void updateMap(const std::vector<std::vector<bool>>& new_map);
    
    // 맵 정보 가져오기
    const std::vector<std::vector<bool>>& getMap() const { return map_; }
    
    // 경로 유효성 검사
    bool validatePaths(const std::vector<std::vector<Position>>& paths) const;

private:
    std::vector<std::vector<bool>> map_;
    
    // CBS 내부 구현
    std::vector<Conflict> detectCollisions(const std::vector<std::vector<Position>>& paths) const;
    std::vector<Constraint> standardSplitting(const Conflict& conflict) const;
    std::vector<Constraint> disjointSplitting(const Conflict& conflict) const;
    
    // 휴리스틱 계산
    std::vector<std::vector<int>> calculateManhattanHeuristics(const Position& goal) const;
};