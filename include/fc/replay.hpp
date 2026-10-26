#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "fc/flight_log.hpp"

namespace fc::replay {

struct Mismatch {
    std::uint32_t tick = 0;
    std::string field;
    double expected = 0.0;
    double actual = 0.0;
};

struct Transition {
    std::uint32_t tick = 0;
    FlightState state = FlightState::Pad;
};

struct Result {
    bool ok = false;     // log read cleanly and every frame matched
    std::string error;   // log read error, if any
    std::size_t ticks = 0;
    std::size_t mismatched_ticks = 0;
    std::optional<Mismatch> first_mismatch;
    std::vector<Transition> transitions;
    FlightState final_state = FlightState::Pad;
    std::uint16_t faults_seen = 0;
};

// Compares a recomputed frame against the recorded one. Discrete fields must match
// exactly; floats within `tol` (relative, with an absolute floor of `tol`).
std::optional<Mismatch> compare(const flightlog::TickRecord& expected, const FcOutputs& actual, double tol);

// Re-runs every recorded frame of `path` through a fresh FlightComputer built from
// the config stored in the log, and checks the outputs against the recording.
// `override_cfg`, if given, replaces the logged config (to test what a change does).
Result run(const std::string& path, double tol = 1e-4, const FcConfig* override_cfg = nullptr);

}  // namespace fc::replay
