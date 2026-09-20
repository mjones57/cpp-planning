#pragma once
#include "planning/bfs.hpp"
#include "planning/search_stats.hpp"

namespace planning
{
    enum class Connectivity { Four, Eight };
    std::vector<Cell> dijkstra(const GridMap &, Cell, Cell, 
                                SearchStats* stats = nullptr,
                                Connectivity conn = Connectivity::Four);
    std::vector<Cell> dijkstra_set(const GridMap &, Cell, Cell, 
                                    SearchStats* stats = nullptr,
                                    Connectivity conn = Connectivity::Four);
}