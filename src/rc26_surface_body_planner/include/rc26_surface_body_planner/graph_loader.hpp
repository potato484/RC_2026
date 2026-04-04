#pragma once

#include "rc26_surface_body_planner/types.hpp"

#include <string>
#include <vector>

namespace rc26_surface_body_planner {

struct LoadResult {
    bool success = false;
    std::string error;
    SurfaceGraph graph;
};

struct ValidationResult {
    bool valid = true;
    std::vector<std::string> errors;
};

LoadResult loadSurfaceGraph(const std::string& yaml_path);
ValidationResult validateSurfaceGraph(const SurfaceGraph& graph);

}  // namespace rc26_surface_body_planner
