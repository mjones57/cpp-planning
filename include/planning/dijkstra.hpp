#pragma once
#include "planning/bfs.hpp"

namespace planning
{
    std::vector<Cell> dijkstra(const GridMap &, Cell, Cell, long *expansions = nullptr);
}