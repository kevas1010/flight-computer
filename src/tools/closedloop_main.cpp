// Runs a closed-loop flight in one process (no sockets, no wall clock) and prints
// the outcome. Used to tune the vehicle, explore faults, and record golden logs:
//
//   fc_closedloop --scenario imu_stuck --log tests/data/imu_stuck.fclog
//   fc_closedloop --all

#include <cstdio>
#include <cstring>
#include <string>

#include "sim/closed_loop.hpp"

namespace {

void print_result(const sim::Scenario& sc, const sim::ScenarioResult& r) {
    std::printf("scenario %-16s final=%-12s", sc.name.c_str(), fc::to_string(r.final_state));
    if (r.final_state == fc::FlightState::Abort) std::printf(" reason=%s", fc::to_string(r.abort_reason));
    std::printf("\n  touchdown %.2f m/s  apogee %.1f m  t_end %.2f s  prop_left %.1f kg  max_nav_err %.2f m  faults_seen 0x%04x%s\n",
                r.touchdown_speed, r.max_altitude, r.end_time_s, r.prop_remaining_kg, r.max_nav_error_m, r.faults_seen,
                r.chute ? "  chute" : "");
    std::printf("  frames with alt_stale %d, imu fault %d\n", r.alt_stale_frames, r.imu_fault_frames);
    std::printf("  transitions:");
    for (const auto& t : r.transitions) std::printf(" %s@%.2f", fc::to_string(t.state), t.time_s);
    std::printf("\n");
}

void usage() {
    std::fprintf(stderr, "usage: fc_closedloop [--scenario NAME] [--seed N] [--log PATH] [--all]\nscenarios:");
    for (const auto& n : sim::scenario_names()) std::fprintf(stderr, " %s", n.c_str());
    std::fprintf(stderr, "\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string name = "nominal";
    std::string log_path;
    long seed = -1;
    bool all = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            name = argv[++i];
        } else if (arg == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (arg == "--seed" && i + 1 < argc) {
            seed = std::stol(argv[++i]);
        } else if (arg == "--all") {
            all = true;
        } else {
            usage();
            return 2;
        }
    }

    if (all) {
        for (const auto& n : sim::scenario_names()) {
            auto sc = *sim::named_scenario(n);
            if (seed >= 0) sc.seed = static_cast<std::uint64_t>(seed);
            print_result(sc, sim::run_closed_loop(sc));
        }
        return 0;
    }

    auto sc = sim::named_scenario(name);
    if (!sc) {
        usage();
        return 2;
    }
    if (seed >= 0) sc->seed = static_cast<std::uint64_t>(seed);
    const auto r = sim::run_closed_loop(*sc, {}, log_path);
    print_result(*sc, r);
    if (!log_path.empty()) std::printf("  log written to %s\n", log_path.c_str());
    return 0;
}
