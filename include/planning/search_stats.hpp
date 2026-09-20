#pragma once
#include <cstddef>

namespace planning
{

    struct SearchStats 
    {
        long expansions     = 0;
        long inserts        = 0;    // pushes or inserts
        long removals       = 0;    // stale pops or decrease-key erases
        std::size_t peak    = 0;    // largest the frontier ever gets
    };  
    
}