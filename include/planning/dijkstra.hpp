#pragma once
#include "planning/grid_map.hpp"
#include "planning/search_stats.hpp"
#include "planning/planner.hpp"

namespace planning
{
    enum class Connectivity
    {
        Four,
        Eight
    };

    enum class Frontier
    {
        Heap,
        Set
    };

    std::vector<Cell> dijkstra(const GridMap &, Cell, Cell,
                               SearchStats *stats = nullptr,
                               Connectivity conn = Connectivity::Four);
    std::vector<Cell> dijkstra_set(const GridMap &, Cell, Cell,
                                   SearchStats *stats = nullptr,
                                   Connectivity conn = Connectivity::Four);

    class DijkstraPlanner final : public Planner
    {
    public:
        explicit DijkstraPlanner(Connectivity conn = Connectivity::Four,
                                 Frontier frontier = Frontier::Heap)
            : conn_(conn), frontier_(frontier) {}

        void configure(const GridMap &map) override { map_ = &map; }
        std::vector<Cell> createPlan(Cell s, Cell g) override
        {
            stats_ = SearchStats{};
            return frontier_ == Frontier::Heap
                       ? dijkstra(*map_, s, g, &stats_, conn_)
                       : dijkstra_set(*map_, s, g, &stats_, conn_);
        }
        std::string name() const override
        {
            std::string n = "Dijkstra (";
            n += (conn_ == Connectivity::Four ? "4-conn" : "8-conn");
            n += (frontier_ == Frontier::Heap ? ", heap)" : ", set)");
            return n;
        }

    private:
        Connectivity conn_;
        Frontier frontier_;
    };
}