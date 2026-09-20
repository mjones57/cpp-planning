#include "planning/dijkstra.hpp"
#include "search_common.hpp"
#include <iostream>
#include <queue>
#include <limits>
#include <set>

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

        // structs for use with dijkstra_set():
        struct Entry { int g; std::size_t idx; };
        struct ByCostThenIdx
        {
            bool operator()(const Entry& a, const Entry& b) const 
            {
                if (a.g != b.g) return a.g < b.g;
                return a.idx < b.idx;   // tie-break to keep distinct cells distinct
            }
        };
        
    }


    // Four-connected version
    std::vector<Cell> dijkstra(const GridMap &m, Cell start, Cell goal, SearchStats* stats, Connectivity conn)
    {
        // Guard endpoints
        if (!m.is_free(start))
            return {};
        if (!m.is_free(goal))
            return {};
        
        SearchStats local;

        // Init cost_so_far, parent, frontier
        std::vector<int> cost_so_far(m.size(), kInf);

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
            if (cur.cost > cost_so_far[ci]) { ++local.removals; continue; }

            // Increment expansion count (after stale guard)
            ++local.expansions;

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


    // version of dijkstra using set instead of priority_queue
    std::vector<Cell> dijkstra_set(const GridMap &m, Cell start, Cell goal, SearchStats* stats, Connectivity conn)
    {
        // Guard endpoints
        if (!m.is_free(start))
            return {};
        if (!m.is_free(goal))
            return {};
        
        SearchStats local;

        // Init cost_so_far, parent, frontier
        std::vector<int> cost_so_far(m.size(), kInf);

        std::vector<int> parent(m.size(), 0);

        std::set<Entry, ByCostThenIdx> frontier;
        frontier.insert(Entry{0, m.index(start)});

        std::size_t start_index = m.index(start);
        cost_so_far[m.index(start)] = 1;
        parent[start_index] = -1;

        bool goal_found = false;

        while (!frontier.empty())
        {

            // Get current node and stale guard entry
            const Entry cur = *frontier.begin();    // COPY, not a reference
            frontier.erase(frontier.begin());
            const Cell cur_cell = m.cell_at(cur.idx);

            // Increment expansion count
            ++local.expansions;

            // early exit condition: reached goal
            if (cur_cell == goal)
            {
                goal_found = true;
                break;
            }

            // get neighbors
            const std::vector<Cell> ns = (conn == Connectivity::Eight) ? neighbors8(m, cur_cell)
                                                                       : neighbors4(m, cur_cell);

            // loop through neighbors
            for (const Cell &n : ns)
            {
                // Relaxation step
                std::size_t ni{m.index(n)};
                const int new_cost = cur.g + edge_cost(m, cur_cell, n);
                if (new_cost < cost_so_far[ni])
                {
                    if (cost_so_far[ni] != kInf)
                    {
                        frontier.erase({cost_so_far[ni], ni});  // OLD KEY, must read it first
                        ++local.removals;
                    }
                    cost_so_far[ni] = new_cost;
                    parent[ni] = static_cast<int>(cur.idx);
                    frontier.insert({new_cost, ni});
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

}