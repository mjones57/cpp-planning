#include "planning/bfs.hpp"
#include "planning/dijkstra.hpp"
#include "planning/astar.hpp"
#include "planning/planner_factory.hpp"

namespace planning
{
    std::unique_ptr<Planner> make_planner(const std::string &id)
    {
        // creates planner for registry using string id
        if (id == "bfs")
            return std::make_unique<BfsPlanner>();
        if (id == "astar")
            return std::make_unique<AStarPlanner>();
        if (id == "weighted")
            return std::make_unique<AStarPlanner>(3);
        if (id == "dijkstra4")
            return std::make_unique<DijkstraPlanner>();
        if (id == "dijkstra8")
            return std::make_unique<DijkstraPlanner>(Connectivity::Eight);
        if (id == "dijkstra4_set")
            return std::make_unique<DijkstraPlanner>(Connectivity::Four, Frontier::Set);
        if (id == "dijkstra8_set")
            return std::make_unique<DijkstraPlanner>(Connectivity::Eight, Frontier::Set);
        return nullptr; // unknown id fallback
    }
}