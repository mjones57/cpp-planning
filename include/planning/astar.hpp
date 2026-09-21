#pragma once
#include "planning/grid_map.hpp"
#include "planning/search_stats.hpp"
#include "planning/planner.hpp"

namespace planning
{
    int manhattan(Cell, Cell);
    int heuristic(const GridMap &, Cell, Cell);
    std::vector<Cell> astar(const GridMap &, Cell, Cell, SearchStats *stats = nullptr, int weight = 1);

    class AStarPlanner final : public Planner
    {
    public:
        explicit AStarPlanner(int weight = 1) : weight_(weight) {}

        void configure(const GridMap &map) override { map_ = &map; }
        std::vector<Cell> createPlan(Cell s, Cell g) override
        {
            stats_ = SearchStats{};
            return astar(*map_, s, g, &stats_, weight_);
        }
        std::string name() const override
        {
            return weight_ == 1 ? "A*" : "Weighted A*";
        }

    private:
        int weight_ = 1;
    };
}