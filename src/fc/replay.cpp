#include "fc/replay.hpp"

#include <algorithm>
#include <cmath>

namespace fc::replay {
namespace {

bool close_enough(double a, double b, double tol) {
    if (a == b) return true;  // also covers matching infinities
    if (std::isnan(a) && std::isnan(b)) return true;
    return std::abs(a - b) <= tol * std::max({1.0, std::abs(a), std::abs(b)});
}

}  // namespace

std::optional<Mismatch> compare(const flightlog::TickRecord& e, const FcOutputs& a, double tol) {
    const auto exact = [&](const char* name, double ev, double av) -> std::optional<Mismatch> {
        if (ev != av) return Mismatch{e.tick, name, ev, av};
        return std::nullopt;
    };
    const auto approx = [&](const char* name, double ev, double av) -> std::optional<Mismatch> {
        if (!close_enough(ev, av, tol)) return Mismatch{e.tick, name, ev, av};
        return std::nullopt;
    };
    const proto::Telemetry& et = e.telemetry;
    const proto::Telemetry& at = a.telemetry;

    for (const auto& m : {
             exact("state", et.state, at.state),
             exact("faults", et.faults, at.faults),
             exact("engine_on", e.actuator.engine_on, a.actuator.engine_on),
             exact("chute", e.actuator.chute, a.actuator.chute),
             approx("throttle", e.actuator.throttle, a.actuator.throttle),
             approx("h_est", et.h_est, at.h_est),
             approx("v_est", et.v_est, at.v_est),
             approx("bias_est", et.bias_est, at.bias_est),
             approx("h_sigma", et.h_sigma, at.h_sigma),
             approx("mass_est", et.mass_est, at.mass_est),
             approx("ignition_alt", et.ignition_alt, at.ignition_alt),
             exact("cmds_accepted", et.cmds_accepted, at.cmds_accepted),
             exact("cmds_duplicate", et.cmds_duplicate, at.cmds_duplicate),
             exact("alt_rejects", et.alt_rejects, at.alt_rejects),
             exact("n_acks", static_cast<double>(e.acks.size()), static_cast<double>(a.n_acks)),
         }) {
        if (m) return m;
    }
    for (std::size_t i = 0; i < e.acks.size(); ++i) {
        if (e.acks[i].cmd_id != a.acks[i].cmd_id) return Mismatch{e.tick, "ack.cmd_id", 1.0 * e.acks[i].cmd_id, 1.0 * a.acks[i].cmd_id};
        if (e.acks[i].result != a.acks[i].result)
            return Mismatch{e.tick, "ack.result", 1.0 * static_cast<int>(e.acks[i].result),
                            1.0 * static_cast<int>(a.acks[i].result)};
    }
    return std::nullopt;
}

Result run(const std::string& path, double tol, const FcConfig* override_cfg) {
    Result res;
    flightlog::Reader reader;
    FcConfig cfg;
    if (!reader.open(path, cfg)) {
        res.error = reader.error();
        return res;
    }
    if (override_cfg != nullptr) cfg = *override_cfg;

    FlightComputer fc(cfg);
    flightlog::TickRecord rec;
    FlightState last = FlightState::Pad;
    while (reader.next(rec)) {
        const FcOutputs& out = fc.step({rec.sensors, rec.commands});
        ++res.ticks;
        res.faults_seen |= out.telemetry.faults;
        const auto st = static_cast<FlightState>(out.telemetry.state);
        if (st != last) {
            res.transitions.push_back({rec.tick, st});
            last = st;
        }
        if (auto m = compare(rec, out, tol)) {
            ++res.mismatched_ticks;
            if (!res.first_mismatch) res.first_mismatch = m;
        }
    }
    res.error = reader.error();
    res.final_state = last;
    res.ok = res.error.empty() && res.ticks > 0 && res.mismatched_ticks == 0;
    return res;
}

}  // namespace fc::replay
