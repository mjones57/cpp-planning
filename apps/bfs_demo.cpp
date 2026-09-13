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
    int expansions1{0};
    std::vector<planning::Cell> path1 = planning::bfs(m, {0, 0}, {9, 5}, expansions1);
    std::cout << "Path 1 length: " << path1.size() << "\n";
    std::cout << "Expansions: " << expansions1 << "\n";
    print_path(m, path1);

    int expansions2{0};
    std::vector<planning::Cell> path2 = planning::bfs(m, {0, 0}, {4, 2}, expansions2);
    std::cout << "Path 2 length: " << path2.size() << "\n";
    std::cout << "Expansions: " << expansions2 << "\n";
    print_path(m, path2);

    return 0;
}