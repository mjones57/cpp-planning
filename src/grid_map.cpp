#include "planning/grid_map.hpp"
#include <array>
#include <iostream>
#include <cassert>
#include <limits>

namespace planning
{
    namespace 
    {
        int min_free_cost(const std::vector<uint8_t>& cells, const std::vector<int>& cost)
        {
            assert(cells.size() == cost.size());
            int best{std::numeric_limits<int>::max()};

            // Check minimum from cells, if its 1 => no free cells
            for (std::size_t i = 0; i < cells.size(); ++i)
            {
                if (cells[i] != 0)
                    continue; // skip if obstacle
                best = std::min(best, cost[i]);
            }

            return best == std::numeric_limits<int>::max() ? 1 : best;
        }

        char terrain_char(const GridMap& m, Cell c)
        {
            if (!m.is_free(c)) return '#';
            switch (m.cost_at(c))
            {
                case 1: return '.';
                case 3: return '^';
                case 5: return '~';
                default: return '?';
            }
        }
    }

    // GridMap construction and member functions:
    GridMap::GridMap(const std::vector<std::string>& rows)
        : width_(rows.empty() ? 0: static_cast<int>(rows[0].size())),
          height_(static_cast<int>(rows.size())) {
            // ... fill cells_ and cost_ then compute min_cost_
            cells_.reserve(width_ * height_);
            cost_.reserve(width_ * height_);

            for (int y = 0; y < height_; y++)
            { // iterate over each row (i = row)
                for (int x = 0; x < width_; x++)
                { // iterate through current row and push_back cell (j = col)
                    if (rows[y][x] == '.')
                    {
                        cells_.push_back(0);
                        cost_.push_back(1);
                    }
                    else if (rows[y][x] == '~')
                    {
                        cells_.push_back(0);
                        cost_.push_back(5);
                    }
                    else if (rows[y][x] == '^')
                    {
                        cells_.push_back(0);
                        cost_.push_back(3);
                    }
                    else if (rows[y][x] == '#')
                    {
                        cells_.push_back(1);
                        cost_.push_back(1);
                    }
                }
            }

            min_cost_ = min_free_cost(cells_, cost_);

            assert(cells_.size() == cost_.size());
            assert(min_cost_ >= 1);
    }

    // row-major: walk across a row, then down. MEMORIZE this!
    std::size_t GridMap::index(Cell c) const
    {
        // note how grid map uses const T& and the cell struct is passed by val b/c its small
        return static_cast<std::size_t>(c.y) * width_ + c.x;
    }

    Cell GridMap::cell_at(std::size_t i) const
    {
        return Cell{static_cast<int>(i) % width_, static_cast<int>(i) / width_};
    }

    // all four edges: do this before ever calling index
    bool GridMap::in_bounds(Cell c) const
    {
        if (c.x < width_ && c.y < height_ && c.x >= 0 && c.y >= 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // check that its in bounds and cell is 0. short-circuiting ordering matters here
    bool GridMap::is_free(Cell c) const
    {
        if (in_bounds(c) && cells_[index(c)] == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int GridMap::cost_at(Cell c) const
    {
        return cost_[index(c)];
    }


    // Non-member functions:
    void print_grid(const GridMap &m)
    {
        // loop y, then x to go in proper order, then convert and print each char
        for (int y = 0; y < m.height(); y++)
        { // iterate over y
            for (int x = 0; x < m.width(); x++)
            { // iterate over x in current y
                Cell c{x, y};
                if (m.is_free(c) == 0)
                {
                    if (m.cost_at(c) == 1)
                        std::cout << '.';
                    else if (m.cost_at(c) == 3)
                        std::cout << '^';
                    else if (m.cost_at(c) == 5)
                        std::cout << '~';
                }
                else
                {
                    std::cout << '#';
                }
            }
            std::cout << std::endl;
        }
    }

    void print_path(const GridMap &m, const std::vector<Cell> &path)
    {
        // Guard empty path case
        if (path.empty())
        {
            std::cout << "unreachable" << "\n";
            return;
        }

        // init canvas: flat row-major order like m.cells
        std::vector<char> canvas;
        canvas.reserve(m.size());

        // walk grid map cells and convert to char in canvas
        for (int y = 0; y < m.height(); ++y)
        {
            for (int x = 0; x < m.width(); ++x)
            {
                canvas.push_back(terrain_char(m, Cell{x, y}));
            }
        }

        // walk the path vector and overwrite its corresponding location on canvas
        for (const Cell &p : path)
        {
            // get flat index and overwrite
            canvas[m.index(p)] = '*';
        }

        // write start and end
        canvas[m.index(path.front())] = 'S';
        canvas[m.index(path.back())] = 'G';

        // Then do the same as print_grid()
        for (int y = 0; y < m.height(); y++)
        { // iterate over y
            for (int x = 0; x < m.width(); x++)
            { // iterate over x in current y
                std::size_t idx = m.index({x, y});
                std::cout << canvas[idx];
            }
            std::cout << "\n";
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
            if (m.is_free(n))
            {
                neighbors.push_back(n);
            }
        }
        return neighbors;
    }

    std::vector<Cell> neighbors8(const GridMap &m, Cell c)
    {

        // Assign four-connected offsets
        static constexpr std::array<Cell, 8> kOffsets{{
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        }};

        // Init empty vector and reserve 4 cells
        std::vector<Cell> neighbors;
        neighbors.reserve(kOffsets.size());

        // Loop through offsets and check if free
        for (const Cell &d : kOffsets)
        {
            const Cell n{c.x + d.x, c.y + d.y};
            if (m.is_free(n))
            {
                neighbors.push_back(n);
            }
        }
        return neighbors;
    }

    int edge_cost(const GridMap& m, Cell from, Cell to)
    {
        const bool diag = (from.x != to.x) && (from.y != to.y);
        return (diag ? kDiag : kOrtho) * m.cost_at(to);
    }

    int path_cost(const GridMap &m, const std::vector<Cell> &path)
    {
        // Sum cost of every cell AFTER the starting cell
        // Precondition: path assumed to be in start->goal order
        int sum{0};

        for (std::size_t ci = 1; ci < path.size(); ci++)
        {
            sum += edge_cost(m, path[ci-1], path[ci]);
        }

        return sum;
    }
}