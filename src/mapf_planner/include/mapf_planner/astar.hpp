#pragma once

#include <vector>
#include <utility>
#include <queue>
#include <unordered_map>
#include <string>
#include "types.hpp"

// Position, Constraint, Node 등은 types.hpp에 정의되어 있다고 가정합니다.

class AStarPlanner {
public:
    AStarPlanner(const std::vector<std::vector<bool>>& map); // 선언만!

    bool isConstrained(
        const Position& curr,
        const Position& next,
        int timestep,
        int agent,
        const std::vector<Constraint>& constraints
    ) const;

    // heuristics: 각 위치에 대한 휴리스틱 맵 [y][x]
    // agent_id: 이 에이전트의 번호
    // constraints: (다중 에이전트 MAPF용) 제약조건
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
};
