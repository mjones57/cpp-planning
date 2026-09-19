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

        // C++20: compiler writes memberwise == and != for us
        bool operator==(const Cell &) const = default;
    };

    class GridMap
    {
    public:
        GridMap() = default;
        explicit GridMap(const std::vector<std::string>& rows);

        // accessors
        int width() const { return width_; }
        int height() const { return height_; }
        std::size_t size() const { return cells_.size(); }

        // cheapest traversable cell, cached at construction
        int min_cost() const { return min_cost_; }

        // member functions
        std::size_t index(Cell c) const;
        bool in_bounds(Cell c) const;
        bool is_free(Cell c) const;
        int cost_at(Cell c) const;

    private:
        int width_ = 0;
        int height_ = 0;
        std::vector<std::uint8_t> cells_; // row-major; 0 = free, 1 = obstacle
        std::vector<int> cost_;           // cost to ENTER this cell, >= 1
        int min_cost_ = 1;
    };

    // thin factory, so construction reads same as before refactor
    inline GridMap make_grid(const std::vector<std::string>& rows) { return GridMap(rows); }

    // free functions
    std::vector<Cell> neighbors4(const GridMap &m, Cell c);
    void print_grid(GridMap &m);
    int path_cost(const GridMap &, const std::vector<Cell> &);
    void print_path(const GridMap &m, const std::vector<Cell> &path);
}
