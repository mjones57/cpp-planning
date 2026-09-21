#include "planning/bfs.hpp"
#include "planning/dijkstra.hpp"
#include "planning/astar.hpp"
#include "planning/planner.hpp"
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
                  << std::setw(10) << st.expansions
                  << std::setw(10) << st.inserts
                  << std::setw(10) << st.removals
                  << std::setw(10) << st.peak << "\n";
    }

    void print_header()
    {
        std::cout << std::left << std::setw(24) << "planner"
                  << std::right << std::setw(6) << "cells"
                  << std::setw(10) << "cost"
                  << std::setw(10) << "expansions"
                  << std::setw(10) << "inserts"
                  << std::setw(10) << "removals"
                  << std::setw(10) << "peak" << "\n"
                  << std::string(80, '-') << "\n";
    }
}

// main func
int main()
{

    // Given text grid map
    std::vector<std::string> rows = {"............", ".~~~~~~~~~~.", ".~~~~~~~~~~.", ".~~~~~~~~~~.", "............"};

    // Convert text to grid map (occupancy grid)
    const GridMap m = planning::make_grid(rows);

    // Run each planner to compare
    BfsPlanner bfs_p;
    DijkstraPlanner dijk_p;
    AStarPlanner astar_p;
    std::vector<Planner *> planners{&bfs_p, &dijk_p, &astar_p};

    Cell s{0, 2}, g{11, 2};

    print_header();
    for (Planner *p : planners)
    {
        p->configure(m);
        print_row(*p, m, s, g);
    }

    return 0;
}