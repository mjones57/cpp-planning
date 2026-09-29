#pragma once
#include <cstddef>
#include <vector>
#include <cassert>

namespace planning
{
    template <typename State>
    class NodePool
    {
    public:
        struct Node
        {
            State state;
            int parent = -1; // Go by index instead of Node* to avoid heap-use-after-free
            double g = 0.0;
            bool pruned = false;
        };

        const Node &at(int i) const
        {
            assert(valid(i));
            return nodes_[i];
        }

        std::size_t size() const { return nodes_.size(); }

        int add(State state, int parent, double g)
        {
            const bool valid_parent = parent == -1 || (parent >= 0 && parent < static_cast<int>(size()));
            if (!valid_parent)
                return -1;

            nodes_.push_back(Node{state, parent, g, false});
            return static_cast<int>(size() - 1);
        }

        void prune(int ni)
        {
            if (valid(ni))
                nodes_[ni].pruned = true;
        }

        [[nodiscard]] std::vector<int> walk(int from) const
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

        void rewire(int ni, int new_parent)
        {
            bool valid_indices = valid(ni) && valid(new_parent);
            if (valid_indices && !nodes_[ni].pruned)
                nodes_[ni].parent = new_parent;
        }

        bool valid(int i) const { return (i < static_cast<int>(nodes_.size()) && i >= 0); }

    private:
        std::vector<Node> nodes_;
    };
}