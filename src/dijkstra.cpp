#include "planning/dijkstra.hpp"
#include <iostream>
#include <queue>
#include <limits>

namespace planning
{

    namespace
    {
        // Anonymous namespace to hold Node and ByCost
        struct Node
        {
            int cost; // g: cost from start to this cell
            Cell c;
        };

        struct ByCost
        {
            bool operator()(const Node &a, const Node &b) const { return a.cost > b.cost; }
        };
    }


    // Four-connected version
    std::vector<Cell> dijkstra(const GridMap &m, Cell start, Cell goal, long *expansions, Connectivity conn)
    {
        // Guard endpoints
        if (!m.is_free(start))
            return {};
        if (!m.is_free(goal))
            return {};
        if (expansions)
            *expansions = 0;

        // Init cost_so_far, parent, frontier
        std::vector<int> cost_so_far(m.size(), std::numeric_limits<int>::max());

        std::vector<int> parent(m.size(), 0);

        std::priority_queue<Node, std::vector<Node>, ByCost> frontier;
        frontier.push(Node{0, start});

        std::size_t start_index = m.index(start);
        cost_so_far[m.index(start)] = 1;
        parent[start_index] = -1;

        bool goal_found = false;

        while (!frontier.empty())
        {

            // Get current node and stale guard entry
            const Node cur = frontier.top();
            frontier.pop();
            const std::size_t ci = m.index(cur.c);
            if (cur.cost > cost_so_far[ci]) { continue; }

            // Increment expansion count (after stale guard)
            if (expansions)
                ++*expansions;

            // early exit condition: reached goal
            if (cur.c.x == goal.x && cur.c.y == goal.y)
            {
                goal_found = true;
                break;
            }

            // get neighbors
            const std::vector<Cell> ns = (conn == Connectivity::Eight) ? neighbors8(m, cur.c)
                                                                       : neighbors4(m, cur.c);

            // loop through neighbors
            for (const Cell &n : ns)
            {
                // Relaxation step
                std::size_t ni{m.index(n)};
                const int new_cost = cur.cost + edge_cost(m, cur.c, n);
                if (new_cost < cost_so_far[ni])
                {
                    cost_so_far[ni] = new_cost;
                    parent[ni] = static_cast<int>(ci);
                    frontier.push(Node{new_cost, n});
                }
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

}