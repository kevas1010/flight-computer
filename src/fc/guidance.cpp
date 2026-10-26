#include "fc/guidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fc {

double ignition_altitude(const VehicleParams& p, const GuidanceConfig& g, double v, double mass) {
    if (v >= 0.0) return -std::numeric_limits<double>::infinity();
    const double a_net = g.plan_throttle * p.max_thrust_n / mass - kG0;
    if (a_net <= 0.0) return std::numeric_limits<double>::infinity();
    const double v_excess = std::max(-v - g.terminal_velocity, 0.0);
    return v_excess * v_excess / (2.0 * a_net) + g.terminal_alt_m + g.ignition_margin_m;
}

void LandingController::start(const GuidanceConfig& g, double h, double v) {
    const double v_excess = std::max(-v - g.terminal_velocity, 0.5);
    const double dh = std::max(h - g.terminal_alt_m, 1.0);
    a_ref_ = v_excess * v_excess / (2.0 * dh);
}

double LandingController::v_ref(const GuidanceConfig& g, double h) const {
    const double dh = std::max(h - g.terminal_alt_m, 0.0);
    return -std::sqrt(2.0 * a_ref_ * dh) - g.terminal_velocity;
}

double LandingController::throttle(const VehicleParams& p, const GuidanceConfig& g, double h, double v,
                                   double mass) const {
    const double a_ff = h > g.terminal_alt_m ? a_ref_ : 0.0;
    const double a_des = a_ff + g.kp * (v_ref(g, h) - v);
    // T + D - m g = m a_des
    const double thrust = mass * (a_des + kG0) - drag_force(p, h, v, false);
    return std::clamp(thrust / p.max_thrust_n, p.min_throttle, 1.0);
}

}  // namespace fc
