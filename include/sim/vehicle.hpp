#pragma once

#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "fc/protocol.hpp"
#include "fc/vehicle_params.hpp"

namespace sim {

enum class FaultKind { AltDropout, ImuStuck, SensorBlackout };

struct Fault {
    FaultKind kind = FaultKind::AltDropout;
    double start_s = 0.0;
    double duration_s = 0.0;

    bool active(double t) const { return t >= start_s && t < start_s + duration_s; }
};

// Parses "alt_dropout@12.5:4" (kind@start_s:duration_s).
std::optional<Fault> parse_fault(const std::string& spec);
const char* to_string(FaultKind k);

struct SensorModel {
    double accel_sigma = 0.05;  // m/s^2
    double accel_bias = 0.04;   // m/s^2, constant
    double alt_sigma = 0.25;    // m
    int alt_divider = 5;        // altimeter at 100 Hz / 5 = 20 Hz
};

// 1-D vertical vehicle: thrust with first-order engine lag, mass depletion,
// drag in an exponential atmosphere, gravity, and ground contact.
class Vehicle {
public:
    explicit Vehicle(const fc::VehicleParams& p = {}, std::uint64_t seed = 1, const SensorModel& sensors = {});

    void command(const fc::proto::ActuatorCmd& cmd);
    void add_fault(const Fault& f) { faults_.push_back(f); }

    // Advances the physics by dt (the sim runs this at 1 kHz).
    void step(double dt);

    // One IMU/altimeter frame. The accelerometer reports the mean specific force
    // since the previous call, like a real IMU's delta-velocity output, so ground
    // impact shows up even between samples. nullopt during a sensor blackout.
    std::optional<fc::proto::SensorSample> sample();

    double time() const { return t_; }
    double altitude() const { return h_; }
    double velocity() const { return v_; }
    double mass() const { return params_.dry_mass_kg + prop_; }
    double prop() const { return prop_; }
    double thrust() const { return thrust_; }
    bool chute() const { return chute_; }
    bool on_ground() const { return on_ground_; }
    bool has_flown() const { return airborne_; }
    bool touched_down() const { return touched_down_; }
    double touchdown_speed() const { return touchdown_speed_; }
    double max_altitude() const { return max_h_; }

private:
    bool fault_active(FaultKind k) const;

    fc::VehicleParams params_;
    SensorModel sensors_;
    std::vector<Fault> faults_;
    std::mt19937_64 rng_;
    std::normal_distribution<double> unit_{0.0, 1.0};

    double t_ = 0.0;
    double h_ = 0.0;
    double v_ = 0.0;
    double prop_;
    double thrust_ = 0.0;
    double throttle_cmd_ = 0.0;
    bool engine_on_ = false;
    bool chute_ = false;
    bool on_ground_ = true;
    bool airborne_ = false;
    bool touched_down_ = false;
    double touchdown_speed_ = 0.0;
    double max_h_ = 0.0;

    double dv_accum_ = 0.0;  // integral of specific force since the last sample
    double t_accum_ = 0.0;
    std::uint64_t sample_count_ = 0;
    bool stuck_latched_ = false;
    float stuck_value_ = 0.0f;
    float last_accel_ = static_cast<float>(fc::kG0);
};

}  // namespace sim
