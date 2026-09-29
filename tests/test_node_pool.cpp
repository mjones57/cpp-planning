#include <gtest/gtest.h>
#include "planning/grid_map.hpp"
#include "planning/node_pool.hpp"
#include "planning/geometry.hpp"
#include <vector>

using namespace planning;

TEST(NodePool, WalkExpectedHops)
{
    // setup node pool
    constexpr int kN = 5;
    NodePool<Cell> pool;
    pool.add(Cell{0, 0}, -1, 0.0); // root

    // add nodes to pool
    int leaf = -1;
    for (int i = 1; i <= kN; ++i)
    {
        leaf = pool.add(Cell{i, i}, i - 1, double(i));
    }

    // walk back leaf->root
    auto path = pool.walk(leaf);

    // Test the path
    EXPECT_EQ(path, (std::vector<int>{5, 4, 3, 2, 1, 0}));
}

TEST(NodePool, LinksSurviveGrowth)
{
    constexpr int kN = 1000;
    NodePool<Cell> pool;
    std::vector<int> ids{pool.add(Cell{0, 0}, -1, 0.0)};
    for (int i = 1; i <= kN; ++i)
        ids.push_back(pool.add(Cell{i, i}, ids.back(), double(i)));

    for (int i = 1; i <= kN; ++i)
    {
        const auto &n = pool.at(ids[i]);
        ASSERT_EQ(pool.at(n.parent).state, (Cell{i - 1, i - 1})) << "link broken at node " << i;
    }
    EXPECT_EQ(pool.walk(ids.back()).size(), kN + 1u); // one full walk end to end
}

TEST(NodePool, SuccessfulPruneAndRewire)
{
    // Initialize node pool
    constexpr int kN = 5;
    NodePool<Cell> pool;
    std::vector<int> ids{pool.add(Cell{0, 0}, -1, 0.0)};

    // Add a few nodes to pool
    int leaf{-1};
    for (int i = 1; i <= kN; ++i)
    {
        leaf = pool.add(Cell{i, i}, ids.back(), double(i));
        ids.push_back(leaf);
    }

    // Prune and rewire intermediate node
    pool.rewire(ids[5], ids[3]); // change index 5's parent to 3
    pool.prune(ids[4]);          // prune index 4
    EXPECT_TRUE(pool.walk(ids[4]).empty());

    for (int i = 1; i <= kN; ++i)
    {
        const auto &n = pool.at(ids[i]);
        ASSERT_EQ(n.state, (Cell{i, i})) << "node " << i << "'s index changes";
    }
    // Test the path
    auto path = pool.walk(leaf);
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path, (std::vector<int>{5, 3, 2, 1, 0}));
}

TEST(NodePool, BadParentRefused)
{
    NodePool<Cell> pool;
    pool.add(Cell{0, 0}, -1, 0.0); // root

    // Add node with negative parent
    EXPECT_EQ(pool.add(Cell{1, 1}, -5, double(1)), -1);
    EXPECT_EQ(pool.size(), 1);

    // Add node with parent beyond size
    EXPECT_EQ(pool.add(Cell{1, 1}, 99, double(1)), -1);
    EXPECT_EQ(pool.size(), 1);

    // Expect successful add
    EXPECT_EQ(pool.add(Cell{2, 2}, 0, double(2)), 1);
    EXPECT_EQ(pool.size(), 2);
}

TEST(NodePool, WalkCycleReturnsEmpty)
{
    // Initialize node pool
    constexpr int kN = 5;
    NodePool<Cell> pool;
    std::vector<int> ids{pool.add(Cell{0, 0}, -1, 0.0)};

    // Add a few nodes to pool
    int leaf{-1};
    for (int i = 1; i <= kN; ++i)
    {
        leaf = pool.add(Cell{i, i}, ids.back(), double(i));
        ids.push_back(leaf);
    }

    // Check pool size
    ASSERT_FALSE(ids.empty());
    EXPECT_EQ(pool.walk(ids.back()).size(), 6u);

    // Rewire intermediate node, create cycle
    pool.rewire(ids[4], ids[5]); // change index 4's parent to 5

    // Expect walk to return empty due to cycle
    EXPECT_TRUE(pool.walk(ids[5]).empty());
}

TEST(NodePool, SecondStateTypeWalks)
{
    // setup node pool
    constexpr int kN = 5;
    NodePool<Point2> pool;
    pool.add(Point2{0, 0}, -1, 0.0); // root

    // add nodes to pool
    int leaf = -1;
    for (int i = 1; i <= kN; ++i)
    {
        leaf = pool.add(Point2{double(i), double(i)}, i - 1, double(i));
    }

    // verify Point2 is stored properly
    EXPECT_DOUBLE_EQ(pool.at(leaf).state.x, 5.0);

    // walk back leaf->root
    auto path = pool.walk(leaf);

    // Test the path via indices
    EXPECT_EQ(path, (std::vector<int>{5, 4, 3, 2, 1, 0}));
}