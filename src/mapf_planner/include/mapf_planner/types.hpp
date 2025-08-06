#pragma once
#include <vector>
#include <utility>

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
