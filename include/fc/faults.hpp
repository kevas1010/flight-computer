#pragma once

#include <cstdint>

namespace fc {

// Fault bits reported in telemetry (ground/protocol.py mirrors these names).
enum FaultBit : std::uint16_t {
    kFaultAltStale = 1u << 0,     // no valid altimeter reading within alt_stale_ticks
    kFaultImuStuck = 1u << 1,     // accelerometer output frozen
    kFaultSensorLink = 1u << 2,   // no sensor packets within sensor_link_ticks
    kFaultAltOutlier = 1u << 3,   // altimeter reading rejected by the innovation gate recently
    kFaultEstDiverged = 1u << 4,  // altitude uncertainty above max_h_sigma
    kFaultImuInvalid = 1u << 5,   // IMU self-reports invalid
};

inline constexpr std::uint16_t kImuFaults = kFaultImuStuck | kFaultImuInvalid;
// Any of these on the pad blocks ARM and holds a running countdown.
inline constexpr std::uint16_t kArmInhibitFaults =
    kFaultAltStale | kFaultImuStuck | kFaultSensorLink | kFaultEstDiverged | kFaultImuInvalid;

struct FaultConfig {
    int imu_stuck_samples = 5;    // bit-identical consecutive samples before declaring stuck
    int imu_clear_samples = 50;   // changing samples before clearing
    int alt_stale_ticks = 25;     // 250 ms at 100 Hz
    int sensor_link_ticks = 20;   // 200 ms
    int alt_outlier_hold_ticks = 100;
    int alt_force_accept = 10;    // consecutive gate rejections before forcing an update
    double max_h_sigma = 10.0;    // m
};

// Detects a frozen accelerometer: a healthy MEMS IMU never repeats the exact same
// float several samples in a row because of its noise floor.
class ImuStuckDetector {
public:
    ImuStuckDetector(int stuck_samples, int clear_samples)
        : stuck_samples_(stuck_samples), clear_samples_(clear_samples) {}

    // Feeds one sample; returns the current stuck state.
    bool update(float accel);
    bool stuck() const { return stuck_; }

private:
    int stuck_samples_;
    int clear_samples_;
    bool have_last_ = false;
    float last_ = 0.0f;
    int repeats_ = 0;
    int changes_ = 0;
    bool stuck_ = false;
};

}  // namespace fc
