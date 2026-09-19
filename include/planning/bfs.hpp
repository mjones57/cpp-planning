#pragma once
#include <vector>
#include "planning/grid_map.hpp"

namespace planning
{
    std::vector<Cell> bfs(const GridMap &, Cell, Cell, long *expansions = nullptr);
    std::vector<Cell> reconstruct(const GridMap &, const std::vector<int> &, const Cell &);
}