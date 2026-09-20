#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "planning/grid_map.hpp"

namespace planning {

inline constexpr int kInf = std::numeric_limits<int>::max();

inline std::vector<Cell> reconstruct(const GridMap& m,
                                     const std::vector<int>& parent, Cell goal) {
    std::vector<Cell> path;
    for (int i = static_cast<int>(m.index(goal)); i != -1; i = parent[i])
        path.push_back(m.cell_at(static_cast<std::size_t>(i)));
    std::reverse(path.begin(), path.end());
    return path;
}

} 