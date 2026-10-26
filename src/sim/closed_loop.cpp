#include "sim/closed_loop.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "fc/flight_log.hpp"

namespace sim {

namespace {

constexpr double kPhysicsDt = 0.001;
constexpr int kStepsPerFrame = 10;  // 1 kHz physics, 100 Hz flight software
constexpr std::uint32_t kSession = 1;

bool terminal(fc::FlightState s) { return s == fc::FlightState::Landed || s == fc::FlightState::Abort; }

}  // namespace

ScenarioResult run_closed_loop(const Scenario& sc, const fc::FcConfig& cfg, const std::string& log_path) {
    Vehicle vehicle(cfg.vehicle, sc.seed);
    for (const Fault& f : sc.faults) vehicle.add_fault(f);
    fc::FlightComputer computer(cfg);

    fc::flightlog::Writer log;
    if (!log_path.empty()) log.open(log_path, cfg);

    ScenarioResult res;
    std::uint32_t next_cmd_id = 1;
    bool arm_sent = false;
    bool abort_sent = false;
    int settled_frames = 0;
    fc::FlightState last = fc::FlightState::Pad;

    const auto max_steps = static_cast<long>(std::lround(sc.max_time_s / kPhysicsDt));
    for (long step = 1; step <= max_steps; ++step) {
        vehicle.step(kPhysicsDt);
        if (step % kStepsPerFrame != 0) continue;

        std::array<fc::proto::SensorSample, 1> sensors{};
        std::size_t n_sensors = 0;
        if (auto s = vehicle.sample()) sensors[n_sensors++] = *s;

        std::array<fc::proto::Command, 2> cmds{};
        std::size_t n_cmds = 0;
        const double t = vehicle.time();
        if (!arm_sent && t >= sc.arm_time_s - 1e-9) {
            cmds[n_cmds++] = {kSession, next_cmd_id++, fc::proto::Opcode::Arm, 0, 0.0f};
            arm_sent = true;
        }
        if (sc.abort_time_s && !abort_sent && t >= *sc.abort_time_s - 1e-9) {
            cmds[n_cmds++] = {kSession, next_cmd_id++, fc::proto::Opcode::Abort, 0, 0.0f};
            abort_sent = true;
        }

        const fc::FcInputs in{{sensors.data(), n_sensors}, {cmds.data(), n_cmds}};
        const fc::FcOutputs& out = computer.step(in);
        vehicle.command(out.actuator);
        if (log.is_open()) log.write(computer.tick(), in, out);

        if (t > 1.0) {
            const std::uint16_t f = out.telemetry.faults;
            res.faults_seen |= f;
            if ((f & fc::kFaultAltStale) != 0) ++res.alt_stale_frames;
            if ((f & fc::kImuFaults) != 0) ++res.imu_fault_frames;
        }
        if (vehicle.has_flown() && !vehicle.on_ground())
            res.max_nav_error_m = std::max(res.max_nav_error_m, std::abs(computer.filter().h() - vehicle.altitude()));

        const fc::FlightState st = computer.state();
        if (st != last) {
            res.transitions.push_back({t, st});
            last = st;
        }
        settled_frames = (terminal(st) && vehicle.on_ground()) ? settled_frames + 1 : 0;
        if (settled_frames >= 200) break;  // 2 s on the ground in a terminal state
    }

    res.final_state = computer.state();
    res.abort_reason = computer.state_machine().abort_reason();
    res.touched_down = vehicle.touched_down();
    res.touchdown_speed = vehicle.touchdown_speed();
    res.max_altitude = vehicle.max_altitude();
    res.end_time_s = vehicle.time();
    res.prop_remaining_kg = vehicle.prop();
    res.chute = vehicle.chute();
    return res;
}

std::optional<Scenario> named_scenario(const std::string& name) {
    Scenario sc;
    sc.name = name;
    if (name == "nominal") return sc;
    if (name == "alt_dropout") {
        // Altimeter lost for 6 s spanning landing-burn ignition: IMU dead-reckoning.
        sc.faults.push_back({FaultKind::AltDropout, 22.0, 6.0});
        return sc;
    }
    if (name == "imu_stuck") {
        // Accelerometer freezes through the landing burn: thrust-model navigation.
        sc.faults.push_back({FaultKind::ImuStuck, 22.0, 30.0});
        return sc;
    }
    if (name == "dual_fault") {
        // IMU frozen and then the altimeter lost: no navigation source, must abort.
        sc.faults.push_back({FaultKind::ImuStuck, 12.0, 30.0});
        sc.faults.push_back({FaultKind::AltDropout, 13.0, 30.0});
        return sc;
    }
    if (name == "sensor_blackout") {
        sc.faults.push_back({FaultKind::SensorBlackout, 10.0, 1.0});
        return sc;
    }
    if (name == "abort_ascent") {
        sc.abort_time_s = 6.0;
        return sc;
    }
    return std::nullopt;
}

std::vector<std::string> scenario_names() {
    return {"nominal", "alt_dropout", "imu_stuck", "dual_fault", "sensor_blackout", "abort_ascent"};
}

}  // namespace sim
