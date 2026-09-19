#pragma once
#include "planning/bfs.hpp"

namespace planning
{
    enum class Connectivity { Four, Eight };
    std::vector<Cell> dijkstra(const GridMap &, Cell, Cell, 
                                long *expansions = nullptr,
                                Connectivity conn = Connectivity::Four);
}