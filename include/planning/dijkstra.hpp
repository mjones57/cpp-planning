#pragma once
#include "planning/bfs.hpp"

namespace planning {
    int path_cost(const GridMap&, const std::vector<Cell>&);
    std::vector<Cell> dijkstra(const GridMap&, Cell, Cell, int&);
}