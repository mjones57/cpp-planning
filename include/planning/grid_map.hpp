#pragma once
#include <vector>
#include <cstdint>
#include <string>

namespace planning
{
    // structs for grid map
    struct Cell
    {
        int x = 0;
        int y = 0;
    };

    struct GridMap
    {
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> cells;    // row-major; 0 = free, 1 = obstacle
        std::vector<int> cost;              // cost to ENTER this cell, >= 1
    };

    // function declarations
    std::size_t index(const GridMap &m, Cell c);
    bool in_bounds(const GridMap &m, Cell c);
    bool is_free(const GridMap &m, Cell c);
    std::vector<Cell> neighbors4(const GridMap &m, Cell c);
    GridMap make_grid(const std::vector<std::string> &rows);
    void print_grid(GridMap &m);

}
