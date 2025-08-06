#pragma once
#include "types.hpp"
#include "astar.hpp"
#include <vector>

class CBSSolver {
public:
    CBSSolver(const std::vector<std::vector<bool>>& map,
              const std::vector<Position>& starts,
              const std::vector<Position>& goals);

    std::vector<std::vector<Position>> findSolution(bool disjoint);

private:
    std::vector<std::vector<bool>> map_;
    std::vector<Position> starts_;
    std::vector<Position> goals_;
    std::vector<std::vector<std::vector<int>>> heuristics_;

    std::vector<std::vector<int>> computeHeuristic(const Position& goal);
    std::vector<Conflict> detectCollisions(const std::vector<std::vector<Position>>& paths) const;
    Conflict detectCollision(const std::vector<Position>& path1,
                             const std::vector<Position>& path2);

    std::vector<Constraint> standardSplitting(const Conflict& conflict) const;
    std::vector<Constraint> disjointSplitting(const Conflict& conflict) const;
    std::vector<int> pathsViolateConstraint(const Constraint& c,
                                            const std::vector<std::vector<Position>>& paths);
};
