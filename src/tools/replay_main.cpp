// Re-runs a recorded flight log through the flight computer and checks every
// frame against the recording. Exit status 0 = identical behaviour.
//
//   fc_replay flight.fclog [--tol 1e-4]

#include <cstdio>
#include <cstdlib>
#include <string>

#include "fc/replay.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fc_replay LOG [--tol TOLERANCE]\n");
        return 2;
    }
    const std::string path = argv[1];
    double tol = 1e-4;
    for (int i = 2; i < argc; ++i) {
        if (std::string(argv[i]) == "--tol" && i + 1 < argc) tol = std::strtod(argv[++i], nullptr);
    }

    const fc::replay::Result r = fc::replay::run(path, tol);
    if (!r.error.empty()) std::fprintf(stderr, "log error: %s\n", r.error.c_str());

    std::printf("replayed %zu frames (%.2f s)\n", r.ticks, static_cast<double>(r.ticks) * 0.01);
    std::printf("transitions:");
    for (const auto& t : r.transitions) std::printf(" %s@%.2f", fc::to_string(t.state), t.tick * 0.01);
    std::printf("\nfinal state %s, faults seen 0x%04x\n", fc::to_string(r.final_state), r.faults_seen);

    if (r.first_mismatch) {
        const auto& m = *r.first_mismatch;
        std::printf("DIVERGED: %zu/%zu frames differ; first at tick %u: %s expected %.9g got %.9g\n",
                    r.mismatched_ticks, r.ticks, m.tick, m.field.c_str(), m.expected, m.actual);
    } else if (r.ok) {
        std::printf("MATCH: all frames reproduce the recording\n");
    }
    return r.ok ? 0 : 1;
}
