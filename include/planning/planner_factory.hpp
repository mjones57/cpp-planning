#pragma once
#include "planning/planner.hpp"
#include <memory>
#include <string>

namespace planning
{
    std::unique_ptr<Planner> make_planner(const std::string &id);
}