#pragma once
#include "planning/grid_map.hpp"
#include <cmath>

namespace planning
{
    struct Point2
    {
        double x = 0.0;
        double y = 0.0;
    };

    inline Cell cell_of(Point2 p) // which cell contains p?
    {
        return Cell{static_cast<int>(std::floor(p.x)), static_cast<int>(std::floor(p.y))};
    }

    inline Point2 center_of(Cell c) // {c.x + 0.5, cy + 0.5}
    {
        return Point2{c.x + 0.5, c.y + 0.5};
    }

    inline double distance(Point2 a, Point2 b)
    {
        return std::hypot(b.x - a.x, b.y - a.y);
    }
}