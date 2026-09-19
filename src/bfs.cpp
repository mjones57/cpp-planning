#include "planning/bfs.hpp"
#include <iostream>
#include <array>
#include <algorithm>
#include <queue>

namespace planning
{
    // Custom functions
    std::vector<Cell> bfs(const GridMap &m, Cell start, Cell goal, long *expansions)
    {

        // Guard endpoints
        if (!m.is_free(start))
            return {};
        if (!m.is_free(goal))
            return {};
        if (expansions)
            *expansions = 0;

        // Init visited, parent, frontier
        std::vector<std::uint8_t> visited(m.size(), 0);

        std::vector<int> parent(m.size(), 0);

        std::queue<Cell> frontier;
        frontier.push(start);
        std::size_t start_index = m.index(start);
        visited[start_index] = 1;
        parent[start_index] = -1;

        bool goal_found = false;

        while (!frontier.empty())
        {

            // Get current cell in queue
            const Cell current = frontier.front();
            frontier.pop();
            const std::size_t ci{m.index(current)};

            // Increment expansion count
            if (expansions)
                ++*expansions;

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
                std::size_t ni{m.index(n)};
                if (visited[ni])
                    continue;

                // if not visited, add and push next
                visited[ni] = 1;
                parent[ni] = ci;
                frontier.push(n);
            }
        }

        if (!goal_found)
        {
            // Unreachable case
            std::vector<Cell> empty_path;
            return empty_path;
        }

        std::vector<Cell> path{reconstruct(m, parent, goal)};
        return path;
    }

    std::vector<Cell> reconstruct(const GridMap &m, const std::vector<int> &parent, const Cell &goal)
    {
        // Reconstructing the path
        std::vector<Cell> path;

        path.push_back(goal);

        // loop through parent until we hit -1
        while (parent[m.index(path.back())] != -1)
        {
            const Cell curr = path.back();
            const std::size_t curr_index{m.index(curr)};

            const int parent_index{parent[curr_index]}; // flat index, need to inverse to get parent cell

            path.push_back({parent_index % m.width(), parent_index / m.width()});
        }
        std::reverse(path.begin(), path.end());

        return path;
    }

    
}