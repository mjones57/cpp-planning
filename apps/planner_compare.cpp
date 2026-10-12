#include "planning/grid_map.hpp"
#include "planning/search_stats.hpp"
#include "planning/planner.hpp"
#include "planning/planner_factory.hpp"
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

using namespace planning;

namespace
{
    void print_row(Planner &p, const GridMap &m, Cell s, Cell g)
    {
        const auto path = p.createPlan(s, g);
        const SearchStats &st = p.last_stats();
        std::cout << std::left << std::setw(24) << p.name()
                  << std::right << std::setw(6) << path.size()
                  << std::setw(10) << path_cost(m, path)
                  << std::setw(12) << st.expansions
                  << std::setw(10) << st.inserts
                  << std::setw(10) << st.removals
                  << std::setw(10) << st.peak << "\n";
    }

    void print_header()
    {
        std::cout << std::left << std::setw(24) << "planner"
                  << std::right << std::setw(6) << "cells"
                  << std::setw(10) << "cost"
                  << std::setw(12) << "expansions"
                  << std::setw(10) << "inserts"
                  << std::setw(10) << "removals"
                  << std::setw(10) << "peak" << "\n"
                  << std::string(80, '-') << "\n";
    }

    std::vector<std::unique_ptr<Planner>> build_registry()
    {
        std::vector<std::unique_ptr<Planner>> registry;
        for (const char *id : {
                 "bfs",
                 "astar",
                 "weighted",
                 "dijkstra4",
                 "dijkstra8",
                 "dijkstra4_set",
                 "dijkstra8_set",
             })
        {
            auto p = make_planner(id);
            if (!p)
            {
                std::cout << "  unknown planner id: " << id << "\n";
                continue;
            }
            registry.push_back(std::move(p)); // MOVE: p is null afterwards!
        }
        return registry;
    }
}

// main func
int main()
{

    // Given text grid map
    std::vector<std::string> rows = {"............", ".~~~~~~~~~~.", ".~~~~~~~~~~.", ".~~~~~~~~~~.", "............"};

    // Convert text to grid map (occupancy grid)
    const GridMap m = planning::make_grid(rows);

    // Build planner registry
    auto registry = build_registry();
    Cell s{0, 2}, g{11, 2};

    print_header();
    for (const auto &p : registry)
    {
        p->configure(m);
        print_row(*p, m, s, g);
    }

    return 0;
}