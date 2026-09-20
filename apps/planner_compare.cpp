#include "planning/bfs.hpp"
#include "planning/dijkstra.hpp"
#include "planning/astar.hpp"
#include <iostream>

// main func
int main()
{

    // Given text grid map
    std::vector<std::string> rows = {"............", ".~~~~~~~~~~.", ".~~~~~~~~~~.", ".~~~~~~~~~~.", "............"};

    // Convert text to grid map (occupancy grid)
    const planning::GridMap m = planning::make_grid(rows);

    // Run bfs test
    planning::SearchStats stats_bfs;
    std::vector<planning::Cell> path_bfs = planning::bfs(m, {0, 2}, {11, 2}, &stats_bfs);
    int bfs_cost{planning::path_cost(m, path_bfs)};
    std::cout << "BFS       cells=" << path_bfs.size() << " cost=" << bfs_cost << " expansions=" << stats_bfs.expansions << "\n";
    print_path(m, path_bfs);

    planning::SearchStats stats_dijk;
    std::vector<planning::Cell> path_dijk = planning::dijkstra(m, {0, 2}, {11, 2}, &stats_dijk);
    int dijk_cost{planning::path_cost(m, path_dijk)};
    std::cout << "Dijkstra  cells=" << path_dijk.size() << " cost=" << dijk_cost << " expansions=" << stats_dijk.expansions << "\n";
    print_path(m, path_dijk);

    planning::SearchStats stats_astar;
    std::vector<planning::Cell> path_astar = planning::astar(m, {0, 2}, {11, 2}, &stats_astar);
    int astar_cost{planning::path_cost(m, path_astar)};
    std::cout << "A*        cells=" << path_astar.size() << " cost=" << astar_cost << " expansions=" << stats_astar.expansions << "\n";
    print_path(m, path_astar);

    return 0;
}