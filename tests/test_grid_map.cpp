#include <gtest/gtest.h>
#include "planning/grid_map.hpp"
#include <algorithm>

using namespace planning;

namespace
{
    // Neighbor order depends of kOffsets. Sort before comparing to address this
    std::vector<Cell> sorted_cells(std::vector<Cell> v)
    {
        std::sort(v.begin(), v.end(), [](Cell a, Cell b)
                  { return a.y != b.y ? a.y < b.y : a.x < b.x; });
        return v;
    }
}

TEST(GridMap, IndexIsRowMajor)
{
    const GridMap m = make_grid({"....", "....", "...."});
    EXPECT_EQ(m.index(Cell{0, 1}), 4u);
    EXPECT_EQ(m.index(Cell{2, 2}), 10u);
}

TEST(GridMap, InBoundsRejectsEdges)
{
    const GridMap m = make_grid({"....", "....", "...."});
    ASSERT_TRUE(m.in_bounds(Cell{0, 0}));
    ASSERT_TRUE(m.in_bounds(Cell{m.width() - 1, m.height() - 1}));
    EXPECT_FALSE(m.in_bounds(Cell{m.width(), 0}));
    EXPECT_FALSE(m.in_bounds(Cell{0, m.height()}));
    EXPECT_FALSE(m.in_bounds(Cell{-1, 0}));
    EXPECT_FALSE(m.in_bounds(Cell{0, -1}));
}

TEST(GridMap, Neighbors4SkipsObstaclesAndEdges)
{
    const GridMap m = make_grid({"...", ".#.", "..."});
    EXPECT_EQ(neighbors4(m, Cell{0, 0}).size(), 2u); // corner
    EXPECT_EQ(neighbors4(m, Cell{1, 0}).size(), 2u); // top edge w/ obs below
    EXPECT_EQ(neighbors4(m, Cell{1, 1}).size(), 4u); // from inside obs
}

TEST(GridMap, Neighbors4ReturnsCorrectCells)
{
    const GridMap m = make_grid({"...", ".#.", "..."});

    // (0, 0) in top left corner, so only right and down are available
    const std::vector<Cell> expected_corner = {{1, 0}, {0, 1}}; // sorted by (y, x)
    EXPECT_EQ(sorted_cells(neighbors4(m, Cell{0, 0})), expected_corner);

    // (1, 1) in middle, so all 4 neighbors are available
    const std::vector<Cell> expected_mid = {{1, 0}, {0, 1}, {2, 1}, {1, 2}}; // sorted by (y, x)
    EXPECT_EQ(sorted_cells(neighbors4(m, Cell{1, 1})), expected_mid);
}

TEST(GridMap, MinFreeCostIgnoresObstacles)
{
    EXPECT_EQ(make_grid({"....", ".~~.", "...."}).min_cost(), 1);
    EXPECT_EQ(make_grid({"~~~~", "~##~", "~~~~"}).min_cost(), 5);
    EXPECT_EQ(make_grid({"~~^~", "~##~", "~~~~"}).min_cost(), 3);
    EXPECT_EQ(make_grid({"####", "####"}).min_cost(), 1);
    EXPECT_EQ(make_grid({}).min_cost(), 1);
}

TEST(GridMap, PathCostIsDirectional)
{
    const GridMap m = make_grid({"....", ".~~~", "...."});
    const std::vector<Cell> path_fwd = {{0, 0}, {1, 0}, {1, 1}};
    const std::vector<Cell> path_rev = {{1, 1}, {1, 0}, {0, 0}};
    EXPECT_EQ(path_cost(m, path_fwd), 60);
    EXPECT_EQ(path_cost(m, path_rev), 20);
}