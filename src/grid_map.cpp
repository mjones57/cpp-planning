#include "planning/grid_map.hpp"
#include <array>
#include <iostream>
#include <cassert>

namespace planning
{
    // Functions to be used for BFS later on:
    GridMap make_grid(const std::vector<std::string> &rows)
    {
        GridMap m;

        // get width (size) of rows and height (num of rows)
        // may need to edit this in case rows[0] is empty!
        m.width = static_cast<int>(rows[0].size());
        m.height = static_cast<int>(rows.size());

        // reserve space
        m.cells.reserve(m.width * m.height);
        m.cost.reserve(m.width * m.height);

        for (int y = 0; y < m.height; y++)
        { // iterate over each row (i = row)
            for (int x = 0; x < m.width; x++)
            { // iterate through current row and push_back cell (j = col)
                if (rows[y][x] == '.')
                {
                    m.cells.push_back(0);
                    m.cost.push_back(1);
                }
                else if (rows[y][x] == '~')
                {
                    m.cells.push_back(0);
                    m.cost.push_back(5);
                }
                else if (rows[y][x] == '#')
                {
                    m.cells.push_back(1);
                    m.cost.push_back(1);
                }
            }
        }

        assert(m.cells.size() == m.cost.size());
        return m;
    }

    void print_grid(const GridMap &m)
    {

        // loop y, then x to go in proper order, then convert and print each char
        for (int y = 0; y < m.height; y++)
        { // iterate over y
            for (int x = 0; x < m.width; x++)
            { // iterate over x in current y
                Cell c{x, y};
                std::size_t idx{index(m, c)};
                if (m.cells[idx] == 0)
                {
                    if (m.cost[idx] == 1) std::cout << '.';
                    else if (m.cost[idx] > 1) std::cout << '~';
                }
                else
                {
                    std::cout << '#';
                }
            }
            std::cout << std::endl;
        }
    }

    // row-major: walk across a row, then down. MEMORIZE this!
    std::size_t index(const GridMap &m, Cell c)
    {
        // note how grid map uses const T& and the cell struct is passed by val b/c its small
        return static_cast<std::size_t>(c.y) * m.width + c.x;
    }

    // all four edges: do this before ever calling index
    bool in_bounds(const GridMap &m, Cell c)
    {
        if (c.x < m.width && c.y < m.height && c.x >= 0 && c.y >= 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // check that its in bounds and cell is 0. short-circuiting ordering matters here
    bool is_free(const GridMap &m, Cell c)
    {
        if (in_bounds(m, c) && m.cells[index(m, c)] == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // the 4-connected free neighbors. use reserve(4) first
    std::vector<Cell> neighbors4(const GridMap &m, Cell c)
    {

        // Assign four-connected offsets
        static constexpr std::array<Cell, 4> kOffsets{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

        // Init empty vector and reserve 4 cells
        std::vector<Cell> neighbors;
        neighbors.reserve(kOffsets.size());

        // Loop through offsets and check if free
        for (const Cell &d : kOffsets)
        {
            const Cell n{c.x + d.x, c.y + d.y};
            if (is_free(m, n))
            {
                neighbors.push_back(n);
            }
        }
        return neighbors;
    }

}