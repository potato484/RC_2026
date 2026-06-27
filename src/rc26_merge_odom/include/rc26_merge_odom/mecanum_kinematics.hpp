#pragma once

#include <cmath>

namespace rc26_merge_odom {

struct BodyVelocity {
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
};

inline BodyVelocity bodyVelocityFromWheelSpeeds(double wheel_base, double track_width, double v_fl, double v_rl,
                                                double v_rr, double v_fr) {
    const double l_plus_w = (wheel_base + track_width) / 2.0;

    BodyVelocity velocity;
    velocity.vx = (v_fl + v_fr + v_rl + v_rr) / 4.0;
    velocity.vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
    velocity.omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * l_plus_w);
    return velocity;
}

inline void integrateHolonomicBodyVelocity(double& x, double& y, double& yaw, double vx, double vy, double omega,
                                           double dt) {
    const double half_dt = dt / 2.0;
    const double mid_yaw = yaw + omega * half_dt;
    const double cos_yaw = std::cos(mid_yaw);
    const double sin_yaw = std::sin(mid_yaw);

    x += (vx * cos_yaw - vy * sin_yaw) * dt;
    y += (vx * sin_yaw + vy * cos_yaw) * dt;
    yaw += omega * dt;

    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }
}

}  // namespace rc26_merge_odom
