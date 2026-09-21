#pragma once
#include <vector>
#include "planning/grid_map.hpp"
#include "planning/search_stats.hpp"
#include "planning/planner.hpp"

namespace planning
{
    std::vector<Cell> bfs(const GridMap &, Cell, Cell, SearchStats *stats = nullptr);

    class BfsPlanner final : public Planner
    {
    public:
        void configure(const GridMap &map) override { map_ = &map; }
        std::vector<Cell> createPlan(Cell s, Cell g) override
        {
            stats_ = SearchStats{};
            return bfs(*map_, s, g, &stats_);
        }
        std::string name() const override { return "BFS"; }
    };
}