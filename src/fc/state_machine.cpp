#include "fc/state_machine.hpp"

#include <cmath>

#include "fc/faults.hpp"

namespace fc {

const char* to_string(FlightState s) {
    switch (s) {
        case FlightState::Pad: return "PAD";
        case FlightState::Ascent: return "ASCENT";
        case FlightState::Coast: return "COAST";
        case FlightState::Descent: return "DESCENT";
        case FlightState::LandingBurn: return "LANDING_BURN";
        case FlightState::Landed: return "LANDED";
        case FlightState::Abort: return "ABORT";
    }
    return "UNKNOWN";
}

const char* to_string(AbortReason r) {
    switch (r) {
        case AbortReason::None: return "none";
        case AbortReason::Commanded: return "commanded";
        case AbortReason::SensorLink: return "sensor_link";
        case AbortReason::NavLoss: return "nav_loss";
        case AbortReason::EstDiverged: return "est_diverged";
    }
    return "unknown";
}

bool StateMachine::can_arm(std::uint16_t faults) const {
    return state_ == FlightState::Pad && !armed_ && (faults & kArmInhibitFaults) == 0;
}

void StateMachine::enter(FlightState s) {
    state_ = s;
    ticks_in_state_ = 0;
    apogee_count_ = 0;
    landed_count_ = 0;
}

void StateMachine::abort(AbortReason why) {
    abort_reason_ = why;
    armed_ = false;
    enter(FlightState::Abort);
}

FlightState StateMachine::update(const StateMachineInputs& in) {
    ++ticks_in_state_;
    if (state_ == FlightState::Landed || state_ == FlightState::Abort) return state_;
    if (in.abort) {
        abort(AbortReason::Commanded);
        return state_;
    }

    if (state_ == FlightState::Pad) {
        if (armed_) {
            if ((in.faults & kArmInhibitFaults) != 0) {
                // Countdown hold: disarm and wait for the ground to re-arm.
                armed_ = false;
                countdown_ticks_ = 0;
                ++holds_;
            } else if (--countdown_ticks_ <= 0) {
                enter(FlightState::Ascent);
            }
        } else if (in.arm && can_arm(in.faults)) {
            armed_ = true;
            countdown_ticks_ = std::lround(cfg_.countdown_s / cfg_.dt);
        }
        return state_;
    }

    // In flight: navigation health checks first.
    if ((in.faults & kFaultSensorLink) != 0) {
        abort(AbortReason::SensorLink);
        return state_;
    }
    if ((in.faults & kImuFaults) != 0 && (in.faults & kFaultAltStale) != 0) {
        abort(AbortReason::NavLoss);  // no IMU and no altimeter: nothing left to navigate on
        return state_;
    }
    if ((in.faults & kFaultEstDiverged) != 0) {
        abort(AbortReason::EstDiverged);
        return state_;
    }

    switch (state_) {
        case FlightState::Ascent:
            if (seconds_in_state() >= cfg_.ascent_burn_s - 1e-9 || in.prop_kg <= cfg_.landing_reserve_kg)
                enter(FlightState::Coast);
            break;
        case FlightState::Coast:
            apogee_count_ = in.v < 0.0 ? apogee_count_ + 1 : 0;
            if (apogee_count_ >= cfg_.apogee_confirm_ticks) enter(FlightState::Descent);
            break;
        case FlightState::Descent:
            if (in.h <= in.ignition_alt) enter(FlightState::LandingBurn);
            break;
        case FlightState::LandingBurn:
            landed_count_ = (in.h < cfg_.landed_h && std::abs(in.v) < cfg_.landed_v) ? landed_count_ + 1 : 0;
            if (landed_count_ >= cfg_.landed_confirm_ticks) enter(FlightState::Landed);
            break;
        default:
            break;
    }
    return state_;
}

}  // namespace fc
