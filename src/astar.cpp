#include "planning/astar.hpp"
#include "search_common.hpp"
#include <cmath>
#include <queue>

namespace planning
{
    namespace
    {
        // Anonymous namespace to hold Node and ByCost
        struct Node
        {
            int f; // g+h; used for priority queue ordering
            int g; // true cost from start; used for relaxation
            Cell c;
        };

        struct ByCost
        {
            bool operator()(const Node &a, const Node &b) const
            {
                if (a.f == b.f)
                {
                    return a.g < b.g;
                } // tie-breaker: take node with higher g
                return a.f > b.f;
            }
        };
    }

    std::vector<Cell> astar(const GridMap &m, Cell start, Cell goal, SearchStats* stats, int weight)
    {
        // Guard endpoints
        if (!m.is_free(start))
            return {};
        if (!m.is_free(goal))
            return {};
        
        SearchStats local;

        // Init cost_so_far, parent, frontier
        std::vector<int> cost_so_far(m.size(), std::numeric_limits<int>::max());

        std::vector<int> parent(m.size(), 0);

        std::priority_queue<Node, std::vector<Node>, ByCost> frontier;
        frontier.push(Node{heuristic(m, start, goal) * weight, 0, start});

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
            if (cur.g > cost_so_far[ci]) { ++local.removals; continue; }

            // Increment expansion count (after stale guard)
            ++local.expansions;

            // early exit condition: reached goal
            if (cur.c.x == goal.x && cur.c.y == goal.y)
            {
                goal_found = true;
                break;
            }

            // get neighbors
            std::vector<Cell> ns{neighbors4(m, cur.c)};

            // loop through neighbors
            for (const Cell &n : ns)
            {
                // Relaxation step
                std::size_t ni{m.index(n)};
                const int new_g = cur.g + m.cost_at(n);
                if (new_g < cost_so_far[ni])
                {
                    cost_so_far[ni] = new_g;
                    parent[ni] = static_cast<int>(ci);
                    frontier.push(Node{new_g + heuristic(m, n, goal) * weight, new_g, n});
                    ++local.inserts;
                    local.peak = std::max(local.peak, frontier.size());
                }
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

    int manhattan(Cell a, Cell b)
    {
        return abs(a.x - b.x) + abs(a.y - b.y);
    }

    int heuristic(const GridMap &m, Cell from, Cell goal)
    {
        return manhattan(from, goal) * m.min_cost();
    }
}