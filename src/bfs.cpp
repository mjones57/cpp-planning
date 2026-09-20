#include "planning/bfs.hpp"
#include "search_common.hpp"
#include <iostream>
#include <array>
#include <algorithm>
#include <queue>

namespace planning
{
    // Custom functions
    std::vector<Cell> bfs(const GridMap &m, Cell start, Cell goal, SearchStats* stats)
    {

        // Guard endpoints
        if (!m.is_free(start))
            return {};
        if (!m.is_free(goal))
            return {};

        SearchStats local;

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
            ++local.expansions;

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
                ++local.inserts;
                local.peak = std::max(local.peak, frontier.size());
            }
        }

        if (!goal_found)
        {
            // Unreachable case
            std::vector<Cell> empty_path;
            return empty_path;
        }

        if (stats) *stats = local;
        std::vector<Cell> path{reconstruct(m, parent, goal)};
        return path;
    }
    
}