#pragma once
#include "planning/grid_map.hpp"
#include "planning/collision_checker.hpp"
#include <cmath>
#include <algorithm>

namespace planning
{
    inline bool edge_free(const GridMap &m, Point2 a, Point2 b)
    {
        // Check freedom every 0.25 cells along segment
        // Clamp steps in loop to be at least 1
        const int n = std::max(1, static_cast<int>(std::ceil(distance(a, b) / 0.25))); // steps in loop

        // Loop [0, n], checking m.is_free
        Cell prev = cell_of(a);
        for (int i = 0; i <= n; ++i)
        {
            // Interpolation
            const double t = static_cast<double>(i) / n;
            const Point2 p = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
            const Cell cur = cell_of(p);
            if (!m.is_free(cur) || !corner_clear(m, prev, cur))
                return false;
            prev = cur;
        }
        return true;
    }

    class GridCollisionChecker final : public CollisionChecker
    {
    public:
        explicit GridCollisionChecker(const GridMap &m) : map_(&m) {}

        bool segment_free(Point2 a, Point2 b) const override
        {
            return edge_free(*map_, a, b);
        }

        bool point_free(Point2 p) const override
        {
            return map_->is_free(cell_of(p));
        }

    private:
        const GridMap *map_; // non-owning: map must outlive this checker
    };
}