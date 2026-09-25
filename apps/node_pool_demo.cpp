#include "planning/node_pool.hpp"
#include <vector>
#include <iostream>

int main()
{
    planning::NodePool pool; // node pool
    pool.add(planning::Cell{0, 0}, -1, 0.0);
    int added_nodes{20};

    // push new nodes to nodes vector
    int leaf = -1;
    for (int i = 1; i <= added_nodes; ++i)
    {
        leaf = pool.add(planning::Cell{i, i}, i - 1, double(i));
    }

    // walk back from last node to root node
    auto path = pool.walk(leaf);

    // print path
    auto leaf_node = pool.at(leaf);
    std::cout << "walk from (" << leaf_node.c.x << "," << leaf_node.c.y << "):";
    for (const auto &i : path)
    {
        auto n = pool.at(i);
        std::cout << " (" << n.c.x << "," << n.c.y << ")";
    }
    std::cout << "\n";

    return 0;
}