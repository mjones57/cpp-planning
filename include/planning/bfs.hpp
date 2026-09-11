#pragma once
#include <vector>
#include "planning/grid_map.hpp"

namespace planning
{
    std::vector<Cell> bfs(const GridMap &m, Cell start, Cell goal);
    void print_path(const GridMap &m, const std::vector<Cell> &path);
}