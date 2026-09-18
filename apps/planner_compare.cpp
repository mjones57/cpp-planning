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
    long expansions_bfs{0};
    std::vector<planning::Cell> path_bfs = planning::bfs(m, {0, 2}, {11, 2}, &expansions_bfs);
    int bfs_cost{planning::path_cost(m, path_bfs)};
    std::cout << "BFS       cells=" << path_bfs.size() << " cost=" << bfs_cost << " expansions=" << expansions_bfs << "\n";
    print_path(m, path_bfs);

    long expansions_dijk{0};
    std::vector<planning::Cell> path_dijk = planning::dijkstra(m, {0, 2}, {11, 2}, &expansions_dijk);
    int dijk_cost{planning::path_cost(m, path_dijk)};
    std::cout << "Dijkstra  cells=" << path_dijk.size() << " cost=" << dijk_cost << " expansions=" << expansions_dijk << "\n";
    print_path(m, path_dijk);

    long expansions_astar{0};
    std::vector<planning::Cell> path_astar = planning::astar(m, {0, 2}, {11, 2}, &expansions_astar);
    int astar_cost{planning::path_cost(m, path_astar)};
    std::cout << "A*        cells=" << path_astar.size() << " cost=" << astar_cost << " expansions=" << expansions_astar << "\n";
    print_path(m, path_astar);

    return 0;
}