#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "fc/flight_computer.hpp"
#include "sim/vehicle.hpp"

namespace sim {

// A complete flight run in one process: vehicle physics at 1 kHz, flight
// computer at 100 Hz, no sockets and no wall clock. Deterministic for a seed.
struct Scenario {
    std::string name = "nominal";
    std::uint64_t seed = 42;
    double arm_time_s = 1.0;
    std::optional<double> abort_time_s;
    std::vector<Fault> faults;
    double max_time_s = 150.0;
};

struct Transition {
    double time_s = 0.0;
    fc::FlightState state = fc::FlightState::Pad;
};

struct ScenarioResult {
    fc::FlightState final_state = fc::FlightState::Pad;
    fc::AbortReason abort_reason = fc::AbortReason::None;
    bool touched_down = false;
    double touchdown_speed = 0.0;
    double max_altitude = 0.0;
    double end_time_s = 0.0;
    std::uint16_t faults_seen = 0;  // OR of fault bits after the first second (skips start-up)
    int alt_stale_frames = 0;
    int imu_fault_frames = 0;
    double max_nav_error_m = 0.0;   // |h_est - h_true| while airborne
    double prop_remaining_kg = 0.0;
    bool chute = false;
    std::vector<Transition> transitions;
};

ScenarioResult run_closed_loop(const Scenario& sc, const fc::FcConfig& cfg = {}, const std::string& log_path = {});

// Built-in scenarios: nominal, alt_dropout, imu_stuck, dual_fault,
// sensor_blackout, abort_ascent.
std::optional<Scenario> named_scenario(const std::string& name);
std::vector<std::string> scenario_names();

}  // namespace sim
