#pragma once

#include "fc/vehicle_params.hpp"

namespace fc {

struct GuidanceConfig {
    double plan_throttle = 0.75;     // throttle assumed when planning the ignition point
    double ignition_margin_m = 20.0;
    double kp = 1.5;                 // velocity-error gain, 1/s
    double terminal_velocity = 1.0;  // m/s, constant-speed final descent
    double terminal_alt_m = 4.0;     // altitude where the final constant-speed segment begins
};

// Altitude at which a landing burn at `plan_throttle` (ignoring drag, which only
// helps) brings the vehicle to terminal velocity at terminal_alt_m, plus margin.
// -inf while ascending (never ignite), +inf if the vehicle cannot decelerate.
double ignition_altitude(const VehicleParams& p, const GuidanceConfig& g, double v, double mass);

// Tracks the velocity profile v_ref(h) = -sqrt(2 a (h - h_t)) - v_t, where `a`
// is latched at ignition so the profile starts at the vehicle's current state.
class LandingController {
public:
    void start(const GuidanceConfig& g, double h, double v);
    double v_ref(const GuidanceConfig& g, double h) const;
    // Throttle in [min_throttle, 1].
    double throttle(const VehicleParams& p, const GuidanceConfig& g, double h, double v, double mass) const;
    double a_ref() const { return a_ref_; }

private:
    double a_ref_ = 0.0;
};

}  // namespace fc
