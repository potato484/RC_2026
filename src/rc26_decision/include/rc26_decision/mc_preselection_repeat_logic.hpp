#pragma once

#include <algorithm>
#include <cmath>

namespace rc26_decision {

inline double mcPreselectionEffectiveForwardX(double initial_forward_x_m,
                                              double forward_step_m,
                                              int repeat_count) {
  if (!std::isfinite(initial_forward_x_m)) {
    initial_forward_x_m = 0.2;
  }
  if (!std::isfinite(forward_step_m)) {
    forward_step_m = 0.2;
  }
  return initial_forward_x_m +
         forward_step_m * static_cast<double>(std::max(0, repeat_count));
}

} // namespace rc26_decision
