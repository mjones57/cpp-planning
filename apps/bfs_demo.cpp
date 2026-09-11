#include "planning/bfs.hpp"
#include <iostream>

// main func
int main()
{

    // Given text grid map
    std::vector<std::string> rows = {"..........", "..####....", "..#...#...", "..#...#...", "..#####...", ".........."};

    // Convert text to grid map (occupancy grid)
    const planning::GridMap m = planning::make_grid(rows);

    // Run bfs test
    std::vector<planning::Cell> path1 = planning::bfs(m, {0, 0}, {9, 5});
    std::cout << "Path 1 length: " << path1.size() << "\n";
    print_path(m, path1);

    std::vector<planning::Cell> path2 = planning::bfs(m, {0, 0}, {4, 2});
    std::cout << "Path 2 length: " << path2.size() << "\n";
    print_path(m, path2);

    return 0;
}