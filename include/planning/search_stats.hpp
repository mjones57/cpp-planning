#pragma once
#include <cstddef>
#include <ostream>

namespace planning
{

    struct SearchStats
    {
        long expansions = 0;
        long inserts = 0;     // pushes or inserts
        long removals = 0;    // stale pops or decrease-key erases
        std::size_t peak = 0; // largest the frontier ever gets
    };

    inline std::ostream &operator<<(std::ostream &os, const SearchStats &s)
    {
        return os << "exp=" << s.expansions << " ins=" << s.inserts
                  << " rem=" << s.removals << " peak=" << s.peak;
    }
}