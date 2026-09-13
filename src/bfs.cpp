#include "planning/bfs.hpp"
#include <iostream>
#include <array>
#include <algorithm>
#include <queue>

namespace planning
{
    // Custom functions
    std::vector<Cell> bfs(const GridMap &m, Cell start, Cell goal, int& expansions)
    {

        // Guard endpoints
        if (!is_free(m, start))
            return {};
        if (!is_free(m, goal))
            return {};

        // Init visited, parent, frontier
        std::vector<std::uint8_t> visited(m.cells.size(), 0);

        std::vector<int> parent(m.cells.size(), 0);

        std::queue<Cell> frontier;
        frontier.push(start);
        std::size_t start_index = index(m, start);
        visited[start_index] = 1;
        parent[start_index] = -1;

        bool goal_found = false;

        while (!frontier.empty())
        {

            // Get current cell in queue
            const Cell current = frontier.front();
            frontier.pop();
            const std::size_t ci{index(m, current)};

            // Increment expansion count
            ++expansions;

            // early exit condition: reached goal
            if (current.x == goal.x && current.y == goal.y)
            {
                goal_found = true;
                break;
            }

            // get neighbors
            std::vector<Cell> ns{neighbors4(m, current)};

            // loop through neighbors
            for (const Cell &n : ns)
            {
                // check if visited
                std::size_t ni{index(m, n)};
                if (visited[ni])
                    continue;

                // if not visited, add and push next
                visited[ni] = 1;
                parent[ni] = ci;
                frontier.push(n);
            }
        }

        if (!goal_found) {
            // Unreachable case
            std::vector<Cell> empty_path;
            return empty_path;
        }

        std::vector<Cell> path {reconstruct(m, parent, goal)};
        return path;
    }


    std::vector<Cell> reconstruct(const GridMap& m, const std::vector<int>& parent, const Cell& goal)
    {
        // Reconstructing the path
        std::vector<Cell> path;

        path.push_back(goal);

        // loop through parent until we hit -1
        while (parent[index(m, path.back())] != -1)
        {
            const Cell curr = path.back();
            const std::size_t curr_index{index(m, curr)};

            const int parent_index{parent[curr_index]}; // flat index, need to inverse to get parent cell

            path.push_back({parent_index % m.width, parent_index / m.width});
        }
        std::reverse(path.begin(), path.end());
        
        return path;
    }

    void print_path(const GridMap &m, const std::vector<Cell> &path)
    {
        // Guard empty path case
        if (path.empty())
        {
            std::cout << "unreachable" << "\n";
            return;
        }

        // init canvas: flat row-major order like m.cells
        std::vector<char> canvas;
        canvas.reserve(m.cells.size());

        // walk grid map cells and convert to char in canvas
        for (std::size_t i = 0; i < m.cells.size(); i++)
        {
            if (m.cells[i] == 0)
            {
                if (m.cost[i] == 1) canvas.push_back('.');
                else if (m.cost[i] > 1) canvas.push_back('~');
            }
            else
            {
                canvas.push_back('#');
            }
        }

        // walk the path vector and overwrite its corresponding location on canvas
        for (const Cell &p : path)
        {
            // get flat index and overwrite
            canvas[index(m, p)] = '*';
        }

        // write start and end
        canvas[index(m, path.front())] = 'S';
        canvas[index(m, path.back())] = 'G';

        // Then do the same as print_grid()
        for (int y = 0; y < m.height; y++)
        { // iterate over y
            for (int x = 0; x < m.width; x++)
            { // iterate over x in current y
                std::size_t idx = index(m, {x, y});
                std::cout << canvas[idx];
            }
            std::cout << "\n";
        }
    }
}