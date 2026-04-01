#pragma once

#include "rc26_topo_nav/types.hpp"
#include <string>

namespace rc26_topo_nav {

struct LoadResult {
    bool success = false;
    std::string error;
    FieldGraph graph;
};

LoadResult loadFieldGraph(const std::string& yaml_path);

struct ValidationResult {
    bool valid = true;
    std::vector<std::string> errors;
};

ValidationResult validateGraph(const FieldGraph& graph);
ValidationResult validateSymmetry(const FieldGraph& blue, const FieldGraph& red);

}  // namespace rc26_topo_nav
