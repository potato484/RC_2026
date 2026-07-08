#pragma once

#include <algorithm>
#include <cmath>

namespace rc26_decision::navigation {

constexpr double kStraightLinePi = 3.14159265358979323846;

struct StraightLinePose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct StraightLineReference {
  StraightLinePose pose;
  double progress{0.0};
  double length_m{0.0};
};

inline double normalizeAngle(double angle) {
  while (angle > kStraightLinePi) {
    angle -= 2.0 * kStraightLinePi;
  }
  while (angle < -kStraightLinePi) {
    angle += 2.0 * kStraightLinePi;
  }
  return angle;
}

inline double smoothstep(double t) {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

inline StraightLinePose xTurnXTarget(const StraightLinePose &start,
                                     double first_x_m,
                                     double yaw_delta_rad,
                                     double second_x_m) {
  const double target_yaw = normalizeAngle(start.yaw + yaw_delta_rad);
  const double start_c = std::cos(start.yaw);
  const double start_s = std::sin(start.yaw);
  const double target_c = std::cos(target_yaw);
  const double target_s = std::sin(target_yaw);
  return {
      start.x + first_x_m * start_c + second_x_m * target_c,
      start.y + first_x_m * start_s + second_x_m * target_s,
      target_yaw,
  };
}

inline StraightLinePose rotateRetreatTarget(const StraightLinePose &start,
                                            double target_delta_rad,
                                            double retreat_x_m,
                                            double retreat_y_m) {
  const double target_yaw = normalizeAngle(start.yaw + target_delta_rad);
  const double c = std::cos(target_yaw);
  const double s = std::sin(target_yaw);
  return {
      start.x + retreat_x_m * c - retreat_y_m * s,
      start.y + retreat_x_m * s + retreat_y_m * c,
      target_yaw,
  };
}

inline double projectProgressOnLine(const StraightLinePose &start,
                                    const StraightLinePose &target,
                                    double current_x,
                                    double current_y,
                                    double previous_progress) {
  const double dx = target.x - start.x;
  const double dy = target.y - start.y;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= 1.0e-12) {
    return 1.0;
  }
  const double raw_progress =
      ((current_x - start.x) * dx + (current_y - start.y) * dy) / length_sq;
  return std::max(previous_progress, std::clamp(raw_progress, 0.0, 1.0));
}

inline StraightLineReference straightLineReference(
    const StraightLinePose &start, const StraightLinePose &target,
    double progress, double lookahead_m) {
  const double dx = target.x - start.x;
  const double dy = target.y - start.y;
  const double length_m = std::hypot(dx, dy);
  const double path_progress = std::clamp(progress, 0.0, 1.0);
  double reference_progress = path_progress;
  if (length_m > 1.0e-9 && lookahead_m > 0.0) {
    reference_progress =
        std::clamp(reference_progress + lookahead_m / length_m, 0.0, 1.0);
  }

  const double yaw_delta = normalizeAngle(target.yaw - start.yaw);
  const double yaw_progress = smoothstep(path_progress);
  return {
      {
          start.x + dx * reference_progress,
          start.y + dy * reference_progress,
          normalizeAngle(start.yaw + yaw_delta * yaw_progress),
      },
      reference_progress,
      length_m,
  };
}

} // namespace rc26_decision::navigation
