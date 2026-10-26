// Real-time vehicle simulator. Runs physics at 1 kHz, publishes IMU + altimeter
// packets to the flight computer at 100 Hz, and applies the actuator commands it
// receives back.
//
//   fc_sim [--fc 127.0.0.1:5500] [--port 5501] [--seed N]
//          [--fault kind@start:duration]... [--truth truth.csv] [--summary out.json]

#include <array>
#include <csignal>
#include <cstdio>
#include <string>
#include <vector>

#include "fc/protocol.hpp"
#include "fc/rt.hpp"
#include "fc/udp.hpp"
#include "sim/vehicle.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

struct Options {
    std::string fc_endpoint = "127.0.0.1:5500";
    std::uint16_t port = 5501;
    std::uint64_t seed = 42;
    std::vector<sim::Fault> faults;
    std::string truth_path;
    std::string summary_path;
    double max_time_s = 300.0;
};

int usage() {
    std::fprintf(stderr,
                 "usage: fc_sim [--fc HOST:PORT] [--port N] [--seed N] [--fault kind@start:dur]...\n"
                 "              [--truth CSV] [--summary JSON] [--max-time S]\n"
                 "fault kinds: alt_dropout imu_stuck sensor_blackout (times are sim seconds)\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_val = i + 1 < argc;
        if (a == "--fc" && has_val) {
            opt.fc_endpoint = argv[++i];
        } else if (a == "--port" && has_val) {
            opt.port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else if (a == "--seed" && has_val) {
            opt.seed = std::stoull(argv[++i]);
        } else if (a == "--fault" && has_val) {
            const auto f = sim::parse_fault(argv[++i]);
            if (!f) return usage();
            opt.faults.push_back(*f);
        } else if (a == "--truth" && has_val) {
            opt.truth_path = argv[++i];
        } else if (a == "--summary" && has_val) {
            opt.summary_path = argv[++i];
        } else if (a == "--max-time" && has_val) {
            opt.max_time_s = std::stod(argv[++i]);
        } else {
            return usage();
        }
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    fc::net::UdpSocket actuator_rx = fc::net::UdpSocket::bound("127.0.0.1", opt.port);
    fc::net::UdpSocket sensor_tx;
    const sockaddr_in fc_addr = fc::net::parse_endpoint(opt.fc_endpoint);

    sim::Vehicle vehicle({}, opt.seed);
    for (const auto& f : opt.faults) {
        vehicle.add_fault(f);
        std::printf("[sim] fault armed: %s at t=%.2f s for %.2f s\n", sim::to_string(f.kind), f.start_s, f.duration_s);
    }

    std::FILE* truth = opt.truth_path.empty() ? nullptr : std::fopen(opt.truth_path.c_str(), "w");
    if (truth != nullptr) std::fprintf(truth, "t,h,v,mass,thrust,chute\n");

    std::array<std::uint8_t, fc::proto::kMaxPacketSize> buf{};
    std::uint32_t seq = 0;
    std::uint32_t bad_packets = 0;
    std::uint32_t actuator_packets = 0;
    int settled_ms = 0;

    std::printf("[sim] running 1 kHz physics, sensors -> %s, actuators on :%u\n", opt.fc_endpoint.c_str(), opt.port);
    std::fflush(stdout);

    fc::rt::PeriodicTimer timer(1'000'000);  // 1 ms
    for (long step = 1; !g_stop; ++step) {
        timer.wait();

        while (auto n = actuator_rx.recv(buf)) {
            fc::proto::Header h;
            fc::proto::ActuatorCmd cmd;
            if (fc::proto::decode_as(std::span<const std::uint8_t>(buf.data(), *n), h, cmd) ==
                fc::proto::DecodeError::None) {
                vehicle.command(cmd);
                ++actuator_packets;
            } else {
                ++bad_packets;
            }
        }

        vehicle.step(0.001);

        if (step % 10 == 0) {
            if (auto s = vehicle.sample()) {
                const std::size_t len = fc::proto::encode(*s, seq++, s->sim_time_us, buf);
                sensor_tx.send_to({buf.data(), len}, fc_addr);
            }
            if (truth != nullptr)
                std::fprintf(truth, "%.3f,%.4f,%.4f,%.3f,%.1f,%d\n", vehicle.time(), vehicle.altitude(),
                             vehicle.velocity(), vehicle.mass(), vehicle.thrust(), vehicle.chute() ? 1 : 0);
        }

        // Done once the vehicle has flown and is resting on the ground with the engine off.
        const bool at_rest = vehicle.has_flown() && vehicle.on_ground() && vehicle.thrust() < 1.0;
        settled_ms = at_rest ? settled_ms + 1 : 0;
        if (settled_ms >= 3000 || vehicle.time() >= opt.max_time_s) break;
    }

    if (truth != nullptr) std::fclose(truth);

    const bool landed = vehicle.touched_down();
    const double vtd = vehicle.touchdown_speed();
    const char* outcome = !landed ? "NO_TOUCHDOWN" : vehicle.chute() ? "CHUTE_LANDING" : vtd <= 3.0 ? "LANDED" : "CRASH";
    std::printf("[sim] %s: touchdown %.2f m/s, apogee %.1f m, t=%.2f s, prop left %.1f kg, %u actuator pkts, %u bad\n",
                outcome, vtd, vehicle.max_altitude(), vehicle.time(), vehicle.prop(), actuator_packets, bad_packets);

    if (!opt.summary_path.empty()) {
        if (std::FILE* f = std::fopen(opt.summary_path.c_str(), "w")) {
            std::fprintf(f,
                         "{\"outcome\": \"%s\", \"touchdown_speed_mps\": %.3f, \"apogee_m\": %.2f, \"end_time_s\": %.3f, "
                         "\"prop_left_kg\": %.2f, \"chute\": %s, \"actuator_packets\": %u, \"bad_packets\": %u}\n",
                         outcome, vtd, vehicle.max_altitude(), vehicle.time(), vehicle.prop(),
                         vehicle.chute() ? "true" : "false", actuator_packets, bad_packets);
            std::fclose(f);
        }
    }
    return 0;
}
