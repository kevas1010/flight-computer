#pragma once

#include <algorithm>
#include <cmath>

namespace fc {

inline constexpr double kG0 = 9.80665;  // standard gravity, m/s^2

// Nominal vehicle model. The simulator flies the "true" vehicle with these
// values and the flight computer uses the same numbers as its onboard model.
struct VehicleParams {
    double dry_mass_kg = 600.0;
    double prop_mass_kg = 400.0;
    double max_thrust_n = 25000.0;
    double min_throttle = 0.3;        // deep-throttle limit while the engine is lit
    double isp_s = 250.0;
    double engine_tau_s = 0.08;       // first-order thrust response
    double drag_cd = 0.6;
    double ref_area_m2 = 1.0;
    double chute_cd_area_m2 = 350.0;  // Cd*A of the abort parachute

    double mass_flow(double thrust_n) const { return thrust_n / (isp_s * kG0); }
};

// Exponential atmosphere.
inline double air_density(double h_m) { return 1.225 * std::exp(-std::max(h_m, 0.0) / 8500.0); }

// Aerodynamic force along +up; always opposes the velocity.
inline double drag_force(const VehicleParams& p, double h_m, double v_mps, bool chute) {
    const double cd_area = p.drag_cd * p.ref_area_m2 + (chute ? p.chute_cd_area_m2 : 0.0);
    return -0.5 * air_density(h_m) * cd_area * v_mps * std::abs(v_mps);
}

}  // namespace fc
