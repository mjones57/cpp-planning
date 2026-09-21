#include <gtest/gtest.h>
#include "planning/grid_map.hpp"
#include "planning/bfs.hpp"
#include "planning/dijkstra.hpp"
#include "planning/astar.hpp"
#include "planning/planner.hpp"

using namespace planning;
namespace
{
    void ExpectValidPath(const GridMap &m, const std::vector<Cell> &path, Cell start, Cell goal)
    {
        ASSERT_FALSE(path.empty()); // ASSERT: indexing below depends on it
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), goal);
        for (const Cell &c : path)
            EXPECT_TRUE(m.is_free(c)) << "path enters a blocked cell";
        for (std::size_t i = 1; i < path.size(); ++i)
        {
            const int dx = std::abs(path[i].x - path[i - 1].x);
            const int dy = std::abs(path[i].y - path[i - 1].y);
            EXPECT_LE(dx, 1) << "jumps more than one cell between " << i << " and " << i - 1;
            EXPECT_LE(dy, 1) << "jumps more than one cell between  " << i << " and " << i - 1;
            EXPECT_FALSE(dx == 0 && dy == 0) << "duplicate cell locations in path at " << i;
        }
    }

    void ExpectValidPlan(Planner &p, const GridMap &m, Cell s, Cell g)
    {
        p.configure(m);
        ExpectValidPath(m, p.createPlan(s, g), s, g);
    }

    void ExpectNoPath(const GridMap &m, Cell start, Cell goal)
    {
        EXPECT_TRUE(bfs(m, start, goal).empty());
        EXPECT_TRUE(dijkstra(m, start, goal).empty());
        EXPECT_TRUE(astar(m, start, goal).empty());
    }

    const std::vector<std::string> kMudRows = {
        "............",
        ".~~~~~~~~~~.",
        ".~~~~~~~~~~.",
        ".~~~~~~~~~~.",
        "............",
    };

    const std::vector<std::string> kPocketRows = {
        "..........",
        "..####....",
        "..#...#...",
        "..#...#...",
        "..#####...",
        "..........",
    };
}

TEST(Planners, AllReturnStructurallyValidPaths)
{
    const GridMap m = make_grid(kMudRows);
    const Cell s{0, 2}, g{11, 2};
    {
        SCOPED_TRACE("bfs");
        ExpectValidPath(m, bfs(m, s, g), s, g);
    }
    {
        SCOPED_TRACE("dijkstra");
        ExpectValidPath(m, dijkstra(m, s, g), s, g);
    }
    {
        SCOPED_TRACE("astar");
        ExpectValidPath(m, astar(m, s, g), s, g);
    }
}

TEST(Planners, AllReturnStructurallyValidPlans)
{
    const GridMap m = make_grid(kMudRows);
    const Cell s{0, 0}, g{11, 4};
    BfsPlanner bfs_p;
    DijkstraPlanner dijk_p;
    AStarPlanner astar_p;
    std::vector<Planner *> planners{&bfs_p, &dijk_p, &astar_p};

    for (Planner *p : planners)
    {
        SCOPED_TRACE(p->name());
        ExpectValidPlan(*p, m, s, g);
        EXPECT_GT(p->last_stats().expansions, 0) << "stats never populated";
    }
}

TEST(Planners, AStarMatchesDijkstraCost)
{
    const GridMap m = make_grid(kMudRows);
    const Cell s{0, 2}, g{11, 2};
    EXPECT_EQ(path_cost(m, astar(m, s, g)), path_cost(m, dijkstra(m, s, g)));
}

TEST(Planners, AStarExpandsNoMoreThanDijkstra)
{
    const GridMap m = make_grid(kMudRows);
    const Cell s{0, 0}, g{2, 3};
    SearchStats astar_exp{0};
    SearchStats dij_exp{0};
    astar(m, s, g, &astar_exp);
    dijkstra(m, s, g, &dij_exp);
    EXPECT_LE(astar_exp.expansions, dij_exp.expansions);
}

TEST(Planners, HeuristicAdmissibleEverywhere)
{
    const GridMap m = make_grid(kMudRows);
    const Cell g{11, 2};
    for (int y = 0; y < m.height(); ++y)
    {
        for (int x = 0; x < m.width(); ++x)
        {
            const Cell c{x, y};
            if (!m.is_free(c))
                continue;
            const std::vector<Cell> p = dijkstra(m, c, g);
            if (p.empty())
                continue;
            EXPECT_LE(heuristic(m, c, g), path_cost(m, p))
                << "heuristic overestimates at (" << x << "," << y << ")";
        }
    }
}

TEST(Planners, InflatedHeuristicCanBeSuboptimal)
{
    const GridMap m = make_grid(kMudRows);
    const Cell s{0, 2}, g{11, 2};
    const int optimal = path_cost(m, dijkstra(m, s, g));
    const int inflated = path_cost(m, astar(m, s, g, nullptr, 5));
    EXPECT_GT(inflated, optimal);
}

TEST(Planners, DijkstraCostsEqual)
{
    const GridMap m = make_grid(kMudRows);
    const Cell s{0, 2}, g{11, 2};
    const int dij_pq = path_cost(m, dijkstra(m, s, g, nullptr, Connectivity::Eight));
    const int dij_set = path_cost(m, dijkstra_set(m, s, g, nullptr, Connectivity::Eight));
    EXPECT_EQ(dij_pq, dij_set);
}

TEST(Planners, BfsMatchesDijkstraOnUniformCost)
{
    const GridMap m = make_grid(kPocketRows);
    const Cell s{0, 2}, g{11, 2};
    EXPECT_EQ(bfs(m, s, g).size(), dijkstra(m, s, g).size());
}

TEST(Planners, UnreachableGoalReturnsEmptyPath)
{
    ExpectNoPath(make_grid(kPocketRows), Cell{0, 0}, Cell{4, 2});
}

TEST(Planners, BlockedEndpointsReturnEmptyPath)
{
    const GridMap m = make_grid(kPocketRows);
    {
        SCOPED_TRACE("start blocked");
        ExpectNoPath(m, Cell{2, 1}, Cell{9, 5});
    }
    {
        SCOPED_TRACE("goal blocked");
        ExpectNoPath(m, Cell{0, 0}, Cell{2, 1});
    }
}

TEST(Planners, EightConnectedCostNotMoreThanFour)
{
    const GridMap m = make_grid(kPocketRows);
    const Cell s{0, 2}, g{11, 2};
    const std::vector<Cell> path4 = dijkstra(m, s, g);
    const std::vector<Cell> path8 = dijkstra(m, s, g, nullptr, Connectivity::Eight);
    EXPECT_LE(path_cost(m, path8), path_cost(m, path4)) << "8-connected path cost exceeds cost of 4-connected path";
}