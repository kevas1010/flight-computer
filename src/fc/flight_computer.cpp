#include "fc/flight_computer.hpp"

#include <algorithm>
#include <cmath>

namespace fc {

using proto::AckResult;
using proto::Opcode;
using proto::ParamId;

FlightComputer::FlightComputer(const FcConfig& cfg)
    : cfg_(cfg),
      kf_(cfg.kf),
      sm_(cfg.sm),
      imu_stuck_(cfg.faults.imu_stuck_samples, cfg.faults.imu_clear_samples),
      prop_kg_(cfg.vehicle.prop_mass_kg) {}

const FcOutputs& FlightComputer::step(const FcInputs& in) {
    ++tick_;

    // 1. Uplink: execute new commands, re-ACK retransmissions.
    bool arm = false;
    bool abort = false;
    out_.n_acks = 0;
    for (const proto::Command& cmd : in.commands.first(std::min(in.commands.size(), kMaxCommandsPerTick))) {
        proto::Ack ack;
        if (const auto dup = rx_.duplicate_of(cmd)) {
            ack = *dup;
        } else {
            ack = {cmd.session, cmd.cmd_id, cmd.opcode, execute(cmd, arm, abort)};
            rx_.record(ack);
        }
        out_.acks[out_.n_acks++] = ack;
    }

    // 2. Navigation.
    for (const proto::SensorSample& s : in.sensors) process_sample(s);
    faults_ = evaluate_faults();

    // 3. Guidance and mode logic.
    const double mass = cfg_.vehicle.dry_mass_kg + prop_kg_;
    ignition_alt_ = ignition_altitude(cfg_.vehicle, cfg_.guidance, kf_.v(), mass);
    const FlightState prev = sm_.state();
    const FlightState st =
        sm_.update({arm, abort, kf_.h(), kf_.v(), prop_kg_, ignition_alt_, faults_});
    if (st == FlightState::LandingBurn && prev != FlightState::LandingBurn)
        landing_.start(cfg_.guidance, kf_.h(), kf_.v());

    // 4. Control.
    double throttle = 0.0;
    bool engine_on = false;
    switch (st) {
        case FlightState::Ascent:
            engine_on = true;
            throttle = 1.0;
            break;
        case FlightState::LandingBurn:
            engine_on = true;
            throttle = landing_.throttle(cfg_.vehicle, cfg_.guidance, kf_.h(), kf_.v(), mass);
            break;
        case FlightState::Abort:
            chute_ = true;
            break;
        default:
            break;
    }
    if (engine_on && prop_kg_ <= 0.0) engine_on = false;
    model_thrust_n_ = engine_on ? throttle * cfg_.vehicle.max_thrust_n : 0.0;
    prop_kg_ = std::max(0.0, prop_kg_ - cfg_.vehicle.mass_flow(model_thrust_n_) * cfg_.sm.dt);

    out_.actuator.throttle = static_cast<float>(engine_on ? throttle : 0.0);
    out_.actuator.engine_on = engine_on ? 1 : 0;
    out_.actuator.chute = chute_ ? 1 : 0;

    // 5. Telemetry (executive-owned fields are left zero).
    proto::Telemetry& t = out_.telemetry;
    t = {};
    t.tick = tick_;
    t.state = static_cast<std::uint8_t>(st);
    t.faults = faults_;
    t.h_est = static_cast<float>(kf_.h());
    t.v_est = static_cast<float>(kf_.v());
    t.bias_est = static_cast<float>(kf_.bias());
    t.h_sigma = static_cast<float>(kf_.h_sigma());
    t.accel_meas = last_accel_;
    t.alt_meas = last_alt_;
    t.sensor_flags = last_flags_;
    t.throttle_cmd = out_.actuator.throttle;
    t.mass_est = static_cast<float>(cfg_.vehicle.dry_mass_kg + prop_kg_);
    t.ignition_alt = std::isfinite(ignition_alt_) ? static_cast<float>(ignition_alt_) : 0.0f;
    t.cmds_accepted = rx_.accepted();
    t.cmds_duplicate = rx_.duplicates();
    t.alt_rejects = alt_rejects_;
    return out_;
}

void FlightComputer::process_sample(const proto::SensorSample& s) {
    if (have_sample_ && s.sim_time_us <= last_sample_us_) return;  // stale or reordered
    double dt = have_sample_ ? std::min(static_cast<double>(s.sim_time_us - last_sample_us_) * 1e-6, 1.0)
                             : cfg_.sensor_dt_s;
    if (dt > 2.5 * cfg_.sensor_dt_s) {
        // Bridge a sensor gap on the thrust model with honest (large) process noise,
        // in sensor-period steps (drag is stiff); the new IMU sample only describes
        // its own sample period.
        for (; dt > 1.5 * cfg_.sensor_dt_s; dt -= cfg_.sensor_dt_s)
            kf_.predict(model_accel(), cfg_.sensor_dt_s, cfg_.model_accel_sigma, false);
    }
    have_sample_ = true;
    last_sample_us_ = s.sim_time_us;
    last_sensor_tick_ = tick_;
    last_flags_ = s.flags;
    last_accel_ = s.accel_mps2;
    last_imu_valid_ = s.imu_valid();

    const bool imu_ok = s.imu_valid() && !imu_stuck_.update(s.accel_mps2);
    if (imu_ok) {
        kf_.predict(static_cast<double>(s.accel_mps2) - kG0, dt, cfg_.kf.accel_sigma, true);
    } else {
        // Degraded: dead-reckon on the commanded thrust and drag model instead.
        kf_.predict(model_accel(), dt, cfg_.model_accel_sigma, false);
        // The thrust model cannot see the ground reaction force, so once the
        // vehicle is at ground level, constrain it to rest (zero-velocity update).
        const FlightState st = sm_.state();
        const bool at_rest = st == FlightState::Pad || st == FlightState::Landed;
        if (at_rest || (st == FlightState::LandingBurn && kf_.h() < cfg_.zupt_alt_m))
            kf_.update_velocity(0.0, cfg_.zupt_sigma);
    }

    if (s.alt_valid()) {
        have_alt_ = true;
        last_alt_tick_ = tick_;
        last_alt_ = s.alt_m;
        // After a run of rejections, trust the altimeter over a filter that has drifted.
        const bool gate = consecutive_rejects_ < cfg_.faults.alt_force_accept;
        if (kf_.update_altitude(s.alt_m, gate)) {
            consecutive_rejects_ = 0;
        } else {
            ++consecutive_rejects_;
            ++alt_rejects_;
            last_reject_tick_ = tick_;
        }
    }
}

double FlightComputer::model_accel() const {
    const FlightState st = sm_.state();
    if (st == FlightState::Pad || st == FlightState::Landed) return 0.0;  // resting on the ground
    const double mass = cfg_.vehicle.dry_mass_kg + prop_kg_;
    return (model_thrust_n_ + drag_force(cfg_.vehicle, kf_.h(), kf_.v(), chute_)) / mass - kG0;
}

std::uint16_t FlightComputer::evaluate_faults() const {
    std::uint16_t f = 0;
    const auto since = [this](std::uint32_t t) { return static_cast<long>(tick_) - static_cast<long>(t); };
    if (!have_sample_ || since(last_sensor_tick_) > cfg_.faults.sensor_link_ticks) f |= kFaultSensorLink;
    if (!have_alt_ || since(last_alt_tick_) > cfg_.faults.alt_stale_ticks) f |= kFaultAltStale;
    if (imu_stuck_.stuck()) f |= kFaultImuStuck;
    if (have_sample_ && !last_imu_valid_) f |= kFaultImuInvalid;
    if (alt_rejects_ > 0 && since(last_reject_tick_) < cfg_.faults.alt_outlier_hold_ticks) f |= kFaultAltOutlier;
    if (kf_.h_sigma() > cfg_.faults.max_h_sigma) f |= kFaultEstDiverged;
    return f;
}

AckResult FlightComputer::execute(const proto::Command& cmd, bool& arm, bool& abort) {
    switch (cmd.opcode) {
        case Opcode::Arm:
            if (!sm_.can_arm(faults_)) return AckResult::Rejected;
            arm = true;
            return AckResult::Ok;
        case Opcode::Abort:
            if (sm_.state() == FlightState::Landed) return AckResult::Rejected;
            abort = true;
            return AckResult::Ok;
        case Opcode::SetParam:
            return set_param(cmd.param, cmd.value);
        case Opcode::Ping:
            return AckResult::Ok;
    }
    return AckResult::BadOpcode;
}

AckResult FlightComputer::set_param(std::uint8_t id, float value) {
    const double v = value;
    const auto in_range = [v](double lo, double hi) { return std::isfinite(v) && v >= lo && v <= hi; };
    const FlightState st = sm_.state();
    const bool on_pad = st == FlightState::Pad && !sm_.armed();
    const bool before_landing_burn =
        st == FlightState::Pad || st == FlightState::Ascent || st == FlightState::Coast || st == FlightState::Descent;

    switch (static_cast<ParamId>(id)) {
        case ParamId::AscentBurnS:
            if (!in_range(1.0, 30.0)) return AckResult::BadParam;
            if (!on_pad) return AckResult::Rejected;
            sm_.config().ascent_burn_s = v;
            return AckResult::Ok;
        case ParamId::LandingThrottle:
            if (!in_range(0.5, 0.95)) return AckResult::BadParam;
            if (!before_landing_burn) return AckResult::Rejected;
            cfg_.guidance.plan_throttle = v;
            return AckResult::Ok;
        case ParamId::LandingKp:
            if (!in_range(0.1, 5.0)) return AckResult::BadParam;
            cfg_.guidance.kp = v;
            return AckResult::Ok;
        case ParamId::TerminalVelocity:
            if (!in_range(0.5, 5.0)) return AckResult::BadParam;
            if (!before_landing_burn) return AckResult::Rejected;
            cfg_.guidance.terminal_velocity = v;
            return AckResult::Ok;
        case ParamId::IgnitionMarginM:
            if (!in_range(0.0, 200.0)) return AckResult::BadParam;
            if (!before_landing_burn) return AckResult::Rejected;
            cfg_.guidance.ignition_margin_m = v;
            return AckResult::Ok;
    }
    return AckResult::BadParam;
}

}  // namespace fc
