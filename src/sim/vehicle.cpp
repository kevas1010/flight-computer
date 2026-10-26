#include "sim/vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

using fc::kG0;

const char* to_string(FaultKind k) {
    switch (k) {
        case FaultKind::AltDropout: return "alt_dropout";
        case FaultKind::ImuStuck: return "imu_stuck";
        case FaultKind::SensorBlackout: return "sensor_blackout";
    }
    return "unknown";
}

std::optional<Fault> parse_fault(const std::string& spec) {
    const auto at = spec.find('@');
    const auto colon = spec.find(':', at == std::string::npos ? 0 : at);
    if (at == std::string::npos || colon == std::string::npos) return std::nullopt;
    const std::string kind = spec.substr(0, at);
    Fault f;
    if (kind == "alt_dropout") {
        f.kind = FaultKind::AltDropout;
    } else if (kind == "imu_stuck") {
        f.kind = FaultKind::ImuStuck;
    } else if (kind == "sensor_blackout") {
        f.kind = FaultKind::SensorBlackout;
    } else {
        return std::nullopt;
    }
    try {
        f.start_s = std::stod(spec.substr(at + 1, colon - at - 1));
        f.duration_s = std::stod(spec.substr(colon + 1));
    } catch (const std::exception&) {
        return std::nullopt;
    }
    if (f.start_s < 0.0 || f.duration_s <= 0.0) return std::nullopt;
    return f;
}

Vehicle::Vehicle(const fc::VehicleParams& p, std::uint64_t seed, const SensorModel& sensors)
    : params_(p), sensors_(sensors), rng_(seed), prop_(p.prop_mass_kg) {}

void Vehicle::command(const fc::proto::ActuatorCmd& cmd) {
    engine_on_ = cmd.engine_on != 0;
    throttle_cmd_ = cmd.throttle;
    if (cmd.chute != 0) chute_ = true;  // a deployed chute stays deployed
}

bool Vehicle::fault_active(FaultKind k) const {
    return std::any_of(faults_.begin(), faults_.end(), [&](const Fault& f) { return f.kind == k && f.active(t_); });
}

void Vehicle::step(double dt) {
    t_ += dt;

    const bool lit = engine_on_ && prop_ > 0.0;
    const double target = lit ? std::clamp(throttle_cmd_, params_.min_throttle, 1.0) * params_.max_thrust_n : 0.0;
    thrust_ += (target - thrust_) * std::min(dt / params_.engine_tau_s, 1.0);
    if (prop_ <= 0.0) thrust_ = 0.0;
    prop_ = std::max(0.0, prop_ - params_.mass_flow(thrust_) * dt);

    const double m = mass();
    const double a = (thrust_ + fc::drag_force(params_, h_, v_, chute_)) / m - kG0;
    const double v_before = v_;

    if (on_ground_ && a > 0.0) on_ground_ = false;  // liftoff
    if (!on_ground_) {
        v_ += a * dt;
        h_ += v_ * dt;
        if (h_ > 0.05) airborne_ = true;
        if (h_ <= 0.0) {
            if (airborne_ && !touched_down_) {
                touched_down_ = true;
                touchdown_speed_ = -v_;
            }
            h_ = 0.0;
            v_ = 0.0;
            on_ground_ = true;
        }
    }
    max_h_ = std::max(max_h_, h_);

    // Specific force = kinematic acceleration + g (includes the ground contact force).
    dv_accum_ += (v_ - v_before) + kG0 * dt;
    t_accum_ += dt;
}

std::optional<fc::proto::SensorSample> Vehicle::sample() {
    ++sample_count_;
    const double f = t_accum_ > 0.0 ? dv_accum_ / t_accum_ : kG0;
    dv_accum_ = 0.0;
    t_accum_ = 0.0;

    fc::proto::SensorSample s;
    s.sim_time_us = static_cast<std::uint64_t>(std::llround(t_ * 1e6));

    float accel = static_cast<float>(f + sensors_.accel_bias + sensors_.accel_sigma * unit_(rng_));
    if (fault_active(FaultKind::ImuStuck)) {
        if (!stuck_latched_) {
            stuck_latched_ = true;
            stuck_value_ = last_accel_;
        }
        accel = stuck_value_;
    } else {
        stuck_latched_ = false;
    }
    last_accel_ = accel;
    s.accel_mps2 = accel;
    s.flags = fc::proto::kSensorImuValid;

    if (sample_count_ % static_cast<std::uint64_t>(sensors_.alt_divider) == 0) {
        const double noise = sensors_.alt_sigma * unit_(rng_);
        if (!fault_active(FaultKind::AltDropout)) {
            s.alt_m = static_cast<float>(h_ + noise);
            s.flags |= fc::proto::kSensorAltValid;
        }
    }

    if (fault_active(FaultKind::SensorBlackout)) return std::nullopt;
    return s;
}

}  // namespace sim
