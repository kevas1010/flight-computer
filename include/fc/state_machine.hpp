#pragma once

#include <cstdint>
#include <limits>

namespace fc {

enum class FlightState : std::uint8_t {
    Pad = 0,
    Ascent = 1,
    Coast = 2,
    Descent = 3,
    LandingBurn = 4,
    Landed = 5,
    Abort = 6,
};

const char* to_string(FlightState s);

enum class AbortReason : std::uint8_t { None = 0, Commanded, SensorLink, NavLoss, EstDiverged };
const char* to_string(AbortReason r);

struct StateMachineConfig {
    double dt = 0.01;
    double countdown_s = 3.0;
    double ascent_burn_s = 6.0;
    double landing_reserve_kg = 120.0;  // MECO early if propellant drops to this
    int apogee_confirm_ticks = 10;
    double landed_h = 1.0;
    double landed_v = 0.5;
    int landed_confirm_ticks = 10;
};

struct StateMachineInputs {
    bool arm = false;    // validated ARM command this tick
    bool abort = false;  // ABORT command this tick
    double h = 0.0;
    double v = 0.0;
    double prop_kg = 0.0;
    double ignition_alt = -std::numeric_limits<double>::infinity();
    std::uint16_t faults = 0;
};

// PAD -> ASCENT -> COAST -> DESCENT -> LANDING_BURN -> LANDED, with ABORT
// reachable from every non-terminal state. Pure logic: no I/O, no clocks.
class StateMachine {
public:
    explicit StateMachine(const StateMachineConfig& cfg = {}) : cfg_(cfg) {}

    // Advance one tick.
    FlightState update(const StateMachineInputs& in);

    FlightState state() const { return state_; }
    bool armed() const { return armed_; }
    bool can_arm(std::uint16_t faults) const;
    std::uint32_t ticks_in_state() const { return ticks_in_state_; }
    double seconds_in_state() const { return ticks_in_state_ * cfg_.dt; }
    double countdown_remaining_s() const { return countdown_ticks_ * cfg_.dt; }
    std::uint32_t holds() const { return holds_; }
    AbortReason abort_reason() const { return abort_reason_; }

    StateMachineConfig& config() { return cfg_; }
    const StateMachineConfig& config() const { return cfg_; }

private:
    void enter(FlightState s);
    void abort(AbortReason why);

    StateMachineConfig cfg_;
    FlightState state_ = FlightState::Pad;
    std::uint32_t ticks_in_state_ = 0;
    bool armed_ = false;
    long countdown_ticks_ = 0;
    std::uint32_t holds_ = 0;
    int apogee_count_ = 0;
    int landed_count_ = 0;
    AbortReason abort_reason_ = AbortReason::None;
};

}  // namespace fc
