#pragma once
#include <vector>
#include "planning/grid_map.hpp"
#include "planning/search_stats.hpp"

namespace planning
{
    std::vector<Cell> bfs(const GridMap &, Cell, Cell, SearchStats* stats = nullptr);
}