#include "planning/bfs.hpp"
#include "planning/dijkstra.hpp"
#include "planning/astar.hpp"
#include <iomanip>
#include <string>
#include <iostream>

namespace
{
    void print_header() {
        std::cout << std::left  << std::setw(16) << "frontier"
                  << std::right << std::setw(7)  << "cost"
                                << std::setw(8)  << "cells"
                                << std::setw(12) << "expansions"
                                << std::setw(9)  << "inserts"
                                << std::setw(10) << "removals"
                                << std::setw(7)  << "peak" << "\n"
                << std::string(69, '-') << "\n";
    }

    void print_row(const char* label, const planning::GridMap& m,
                   const std::vector<planning::Cell>& path, const planning::SearchStats& s) {
        std::cout << std::left  << std::setw(16) << label
                  << std::right << std::setw(7)  << path_cost(m, path)
                                << std::setw(8)  << path.size()
                                << std::setw(12) << s.expansions
                                << std::setw(9)  << s.inserts
                                << std::setw(10) << s.removals
                                << std::setw(7)  << s.peak << "\n";
    }

    void print_legend() {
        std::cout << "\nremovals: stale pops (wasted work) for the heap,\n"
                << "          decrease-key erases (useful work) for the set.\n"
                << "          The two columns are not directly comparable.\n";
    }
}

int main()
{
    // Given text grid map
    std::vector<std::string> rows = {
        "..........", 
        "..^^^^^^..", 
        "..^~~~~^..", 
        "..^~..~^..", 
        "..^~~~~^..", 
        "..^^^^^^..", 
        ".........."
    };

    // Convert text to grid map (occupancy grid)
    const planning::GridMap m = planning::make_grid(rows);

    // Compare 8-connected dijkstra priority_queue vs dijkstra set
    planning::SearchStats stats_pq;
    std::vector<planning::Cell> path_pq = planning::dijkstra(m, {0, 0}, {9,6}, &stats_pq, planning::Connectivity::Eight);

    planning::SearchStats stats_set;
    std::vector<planning::Cell> path_set = planning::dijkstra_set(m, {0, 0}, {9,6}, &stats_set, planning::Connectivity::Eight);

    print_header();
    print_row("priority_queue", m, path_pq, stats_pq);
    print_row("std::set", m, path_set, stats_set);
    print_legend();

    return 0;
}