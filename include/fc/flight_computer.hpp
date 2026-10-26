#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "fc/command_receiver.hpp"
#include "fc/faults.hpp"
#include "fc/guidance.hpp"
#include "fc/kalman.hpp"
#include "fc/protocol.hpp"
#include "fc/state_machine.hpp"
#include "fc/vehicle_params.hpp"

namespace fc {

struct FcConfig {
    VehicleParams vehicle;
    KalmanConfig kf;
    GuidanceConfig guidance;
    StateMachineConfig sm;
    FaultConfig faults;
    double model_accel_sigma = 1.5;  // process noise when navigating on the thrust model
    double zupt_alt_m = 0.3;         // degraded nav: below this in LANDING_BURN, assume ground contact
    double zupt_sigma = 0.2;         // m/s
    double sensor_dt_s = 0.01;       // nominal sensor period, used for the first sample
};

inline constexpr std::size_t kMaxCommandsPerTick = 8;
inline constexpr std::size_t kMaxSensorsPerTick = 16;

struct FcInputs {
    std::span<const proto::SensorSample> sensors;
    std::span<const proto::Command> commands;
};

struct FcOutputs {
    proto::ActuatorCmd actuator;
    proto::Telemetry telemetry;
    std::array<proto::Ack, kMaxCommandsPerTick> acks{};
    std::size_t n_acks = 0;

    std::span<const proto::Ack> ack_span() const { return {acks.data(), n_acks}; }
};

// The deterministic flight software: one call to step() is one 100 Hz frame.
// It owns no sockets or clocks, so the real-time executive, the in-process
// closed loop and log replay all drive the exact same code.
class FlightComputer {
public:
    explicit FlightComputer(const FcConfig& cfg = {});

    const FcOutputs& step(const FcInputs& in);

    FlightState state() const { return sm_.state(); }
    std::uint16_t faults() const { return faults_; }
    std::uint32_t tick() const { return tick_; }
    double prop_kg() const { return prop_kg_; }
    const AltitudeKalman& filter() const { return kf_; }
    const StateMachine& state_machine() const { return sm_; }
    const FcConfig& config() const { return cfg_; }

private:
    void process_sample(const proto::SensorSample& s);
    double model_accel() const;
    std::uint16_t evaluate_faults() const;
    proto::AckResult execute(const proto::Command& cmd, bool& arm, bool& abort);
    proto::AckResult set_param(std::uint8_t id, float value);

    FcConfig cfg_;
    AltitudeKalman kf_;
    StateMachine sm_;
    LandingController landing_;
    ImuStuckDetector imu_stuck_;
    CommandReceiver rx_;
    FcOutputs out_;

    std::uint32_t tick_ = 0;
    std::uint16_t faults_ = kFaultSensorLink | kFaultAltStale;  // nothing heard yet
    double prop_kg_;
    double model_thrust_n_ = 0.0;
    bool chute_ = false;
    double ignition_alt_ = 0.0;

    bool have_sample_ = false;
    std::uint64_t last_sample_us_ = 0;
    std::uint32_t last_sensor_tick_ = 0;
    bool have_alt_ = false;
    std::uint32_t last_alt_tick_ = 0;
    bool last_imu_valid_ = true;
    float last_accel_ = 0.0f;
    float last_alt_ = 0.0f;
    std::uint8_t last_flags_ = 0;
    int consecutive_rejects_ = 0;
    std::uint32_t alt_rejects_ = 0;
    std::uint32_t last_reject_tick_ = 0;
};

}  // namespace fc
