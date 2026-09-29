#pragma once
#include "planning/geometry.hpp"

/*
Implemented as a class instead of std::function because a checker class can
answer more than one query (point and segment)
*/

namespace planning
{
    class CollisionChecker
    {
    public:
        virtual ~CollisionChecker() = default;

        virtual bool segment_free(Point2 a, Point2 b) const = 0;
        virtual bool point_free(Point2 p) const = 0;
    };
}