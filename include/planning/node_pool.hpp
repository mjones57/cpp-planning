#pragma once
#include "planning/grid_map.hpp"
#include <vector>
#include <cassert>

namespace planning
{
    struct Node
    {
        Cell c;
        int parent = -1; // Go by index instead of Node* to avoid heap-use-after-free
        double g = 0.0;
        bool pruned = false;
    };

    class NodePool
    {
    public:
        const Node &at(int i) const
        {
            assert(valid(i));
            return nodes_[i];
        }
        std::size_t size() const { return nodes_.size(); }
        int add(Cell c, int parent, double g);
        void prune(int ni);
        [[nodiscard]] std::vector<int> walk(int from) const;
        void rewire(int ni, int new_parent);
        bool valid(int i) const { return (i < static_cast<int>(nodes_.size()) && i >= 0); }

    private:
        std::vector<Node> nodes_;
    };
}