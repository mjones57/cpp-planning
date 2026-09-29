#pragma once
#include "planning/grid_map.hpp"
#include "planning/search_stats.hpp"
#include <vector>
#include <string>

/*
I'm keeping this interface grid-only and giving continuous space
planners a separate continuous space entry point. This avoids forcing
continuous space planners to a grid and keeps each interface honest
about its state type.
*/

namespace planning
{
    class Planner
    {
    public:
        virtual ~Planner() = default; // Required step

        virtual void configure(const GridMap &map) = 0;
        virtual std::vector<Cell> createPlan(Cell start, Cell goal) = 0;
        virtual std::string name() const = 0;

        const SearchStats &last_stats() const { return stats_; } // Not virtual

    protected:
        const GridMap *map_ = nullptr; // non-owning, set by configure instead
        SearchStats stats_{};
    };
}