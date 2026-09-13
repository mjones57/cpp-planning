#include "planning/bfs.hpp"
#include "planning/dijkstra.hpp"
#include <iostream>

// main func
int main()
{

    // Given text grid map
    std::vector<std::string> rows = {"............", ".~~~~~~~~~~.", ".~~~~~~~~~~.", ".~~~~~~~~~~.", "............"};

    // Convert text to grid map (occupancy grid)
    const planning::GridMap m = planning::make_grid(rows);

    // Run bfs test
    int expansions_bfs{0};
    std::vector<planning::Cell> path_bfs = planning::bfs(m, {0, 2}, {11, 2}, expansions_bfs);
    int bfs_cost {planning::path_cost(m, path_bfs)};
    std::cout << "BFS       cells=" << path_bfs.size() << " cost=" << bfs_cost << " expansions=" << expansions_bfs << "\n";
    print_path(m, path_bfs);

    int expansions_dijk{0};
    std::vector<planning::Cell> path_dijk = planning::dijkstra(m, {0, 2}, {11, 2}, expansions_dijk);
    int dijk_cost {planning::path_cost(m, path_dijk)};
    std::cout << "Dijkstra  cells=" << path_dijk.size() << " cost=" << dijk_cost << " expansions=" << expansions_dijk << "\n";
    print_path(m, path_dijk);

    return 0;
}