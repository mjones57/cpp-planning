#pragma once
#include "planning/dijkstra.hpp"

namespace planning
{
    int manhattan(Cell, Cell);
    int heuristic(const GridMap &, Cell, Cell);
    std::vector<Cell> astar(const GridMap &, Cell, Cell, long *expansions = nullptr, int weight = 1);
}