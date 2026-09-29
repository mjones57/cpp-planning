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

TEST(GridMap, Neighbors8ReturnsCorrectCells)
{
    const GridMap m1 = make_grid({".#", "#."}); // both ortho and diag blocked
    const GridMap m2 = make_grid({"..", ".."}); // completely free
    const GridMap m3 = make_grid({".#", ".."}); // 3-cell path, diag blocked by one corner

    // map 1 (0,0): expect neighbors8 to return empty (diagonal blocked)
    EXPECT_TRUE(neighbors8(m1, Cell{0, 0}).empty());

    // map 2 (0,0): expect all 3 neighbor cells returned
    const std::vector<Cell> expected_m2 = {{1, 0}, {0, 1}, {1, 1}}; // sorted by (y, x)
    EXPECT_EQ(sorted_cells(neighbors8(m2, Cell{0, 0})), expected_m2);

    // map 3 (0,0): expect only south neighbor
    const std::vector<Cell> expected_m3 = {{0, 1}}; // sorted by (y, x)
    EXPECT_EQ(sorted_cells(neighbors8(m3, Cell{0, 0})), expected_m3);
}