#include "planning/node_pool.hpp"
#include "planning/grid_map.hpp"
#include <vector>

namespace planning
{
    int NodePool::add(Cell c, int parent, double g)
    {
        const bool valid_parent = parent == -1 || (parent >= 0 && parent < static_cast<int>(size()));
        if (!valid_parent)
            return -1;

        nodes_.push_back(Node{c, parent, g, false});
        return static_cast<int>(size() - 1);
    }

    void NodePool::prune(int ni)
    {
        if (valid(ni))
            nodes_[ni].pruned = true;
    }

    [[nodiscard]] std::vector<int> NodePool::walk(int from) const
    {
        std::vector<int> path;

        // walk back from given node to root node
        for (int i = from; i != -1; i = nodes_[i].parent)
        {
            if (path.size() >= size() || nodes_[i].pruned)
            {
                return {};
            }
            path.push_back(i);
        }
        return path;
    }

    void NodePool::rewire(int ni, int new_parent)
    {
        bool valid_indices = valid(ni) && valid(new_parent);
        if (valid_indices && !nodes_[ni].pruned)
            nodes_[ni].parent = new_parent;
    }
}