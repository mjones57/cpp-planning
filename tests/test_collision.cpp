#include <gtest/gtest.h>
#include "planning/grid_map.hpp"
#include "planning/grid_collision_checker.hpp"
#include <vector>
#include <string>

using namespace planning;
namespace
{
    const std::vector<std::string> kWallRows = {
        "....",
        ".##.",
        "....",
    };
}

TEST(CollisionChecker, SegmentFreeDetectsWallsAndBounds)
{
    const GridMap m = make_grid(kWallRows);
    const GridCollisionChecker grid(m);
    const CollisionChecker &cc = grid;                     // make every call below go through interface
    EXPECT_TRUE(cc.segment_free({0.5, 0.5}, {3.5, 0.5}));  // free row
    EXPECT_TRUE(cc.segment_free({0.5, 0.5}, {0.5, 0.5}));  // zero-length segment
    EXPECT_FALSE(cc.segment_free({0.5, 0.5}, {4.5, 0.5})); // end pt out of bounds
    EXPECT_FALSE(cc.segment_free({1.5, 0.5}, {1.5, 2.5})); // segment through wall
}

TEST(CollisionChecker, WallAndNegativeCellsNotFree)
{
    const GridMap m = make_grid(kWallRows);
    const GridCollisionChecker grid(m);
    const CollisionChecker &cc = grid;        // make every call below go through interface
    EXPECT_TRUE(cc.point_free({0.5, 0.5}));   // valid coordinate
    EXPECT_FALSE(cc.point_free({1.5, 1.5}));  // obstacle cell
    EXPECT_FALSE(cc.point_free({-0.5, 1.5})); // negative coordinate
}

TEST(CollisionChecker, SegmentFreeDetectsBlockedDiagonals)
{
    const GridMap m1 = make_grid({".#", "#."}); // both ortho cells blocked
    const GridMap m2 = make_grid({"..", ".."}); // completely free
    const GridMap m3 = make_grid({".#", ".."}); // one ortho cell blocked

    const GridCollisionChecker grid1(m1);
    const CollisionChecker &cc1 = grid1;
    EXPECT_FALSE(cc1.segment_free({0.5, 0.5}, {1.5, 1.5}));

    const GridCollisionChecker grid2(m2);
    const CollisionChecker &cc2 = grid2;
    EXPECT_TRUE(cc2.segment_free({0.5, 0.5}, {1.5, 1.5}));

    const GridCollisionChecker grid3(m3);
    const CollisionChecker &cc3 = grid3;
    EXPECT_FALSE(cc3.segment_free({0.5, 0.5}, {1.5, 1.5}));
}