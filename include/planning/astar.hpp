#pragma once
#include "planning/dijkstra.hpp"
#include "planning/search_stats.hpp"

namespace planning
{
    int manhattan(Cell, Cell);
    int heuristic(const GridMap &, Cell, Cell);
    std::vector<Cell> astar(const GridMap &, Cell, Cell, SearchStats* stats = nullptr, int weight = 1);
}