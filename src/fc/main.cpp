// Flight computer executive: 100 Hz fixed-rate loop around FlightComputer::step().
//
// Each frame: wait for the absolute deadline, drain sensor and uplink sockets,
// step the deterministic core, send actuators to the sim, ACKs to the command
// source and telemetry to the ground station, and append the frame to the log.
//
//   flight_computer [--sensor-port 5500] [--sim 127.0.0.1:5501] [--gs 127.0.0.1:5600]
//                   [--cmd-port 5601] [--log flight.fclog] [--stats stats.json] [--rt]

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <vector>

#include "fc/flight_computer.hpp"
#include "fc/flight_log.hpp"
#include "fc/rt.hpp"
#include "fc/stats.hpp"
#include "fc/udp.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

constexpr std::int64_t kPeriodNs = 10'000'000;  // 100 Hz
constexpr std::size_t kRollingWindow = 100;

struct Options {
    std::uint16_t sensor_port = 5500;
    std::string sim_endpoint = "127.0.0.1:5501";
    std::string gs_endpoint = "127.0.0.1:5600";
    std::uint16_t cmd_port = 5601;
    std::string log_path = "flight.fclog";
    std::string stats_path;
    bool realtime = false;
    int spin_us = 0;
    int tlm_divider = 2;           // 50 Hz telemetry
    double exit_after_terminal_s = 5.0;
    double max_time_s = 600.0;
};

int usage() {
    std::fprintf(stderr,
                 "usage: flight_computer [--sensor-port N] [--sim HOST:PORT] [--gs HOST:PORT] [--cmd-port N]\n"
                 "                       [--log PATH|none] [--stats JSON] [--rt] [--spin-us N] [--tlm-div N]\n"
                 "                       [--exit-after S] [--max-time S]\n");
    return 2;
}

void write_stats(const std::string& path, const fc::LatencySummary& wake, const fc::LatencySummary& jitter,
                 std::uint32_t overruns, std::uint32_t crc_errors, const fc::FlightComputer& fc,
                 const fc::proto::Telemetry& last, bool rt, int spin_us) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) return;
    const auto dump = [f](const char* name, const fc::LatencySummary& s) {
        std::fprintf(f,
                     "  \"%s\": {\"count\": %zu, \"mean_us\": %.2f, \"p50_us\": %.2f, \"p90_us\": %.2f, "
                     "\"p99_us\": %.2f, \"p999_us\": %.2f, \"max_us\": %.2f},\n",
                     name, s.count, s.mean, s.p50, s.p90, s.p99, s.p999, s.max);
    };
    std::fprintf(f, "{\n  \"rate_hz\": 100,\n  \"sched_fifo\": %s,\n  \"spin_us\": %d,\n", rt ? "true" : "false",
                 spin_us);
    dump("wake_latency", wake);
    dump("period_jitter", jitter);
    std::fprintf(f, "  \"overruns\": %u,\n  \"uplink_crc_errors\": %u,\n", overruns, crc_errors);
    std::fprintf(f, "  \"cmds_accepted\": %u,\n  \"cmds_duplicate\": %u,\n", last.cmds_accepted, last.cmds_duplicate);
    std::fprintf(f, "  \"final_state\": \"%s\",\n  \"abort_reason\": \"%s\"\n}\n", fc::to_string(fc.state()),
                 fc::to_string(fc.state_machine().abort_reason()));
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_val = i + 1 < argc;
        if (a == "--sensor-port" && has_val) {
            opt.sensor_port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else if (a == "--sim" && has_val) {
            opt.sim_endpoint = argv[++i];
        } else if (a == "--gs" && has_val) {
            opt.gs_endpoint = argv[++i];
        } else if (a == "--cmd-port" && has_val) {
            opt.cmd_port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else if (a == "--log" && has_val) {
            opt.log_path = argv[++i];
        } else if (a == "--stats" && has_val) {
            opt.stats_path = argv[++i];
        } else if (a == "--rt") {
            opt.realtime = true;
        } else if (a == "--spin-us" && has_val) {
            opt.spin_us = std::max(0, std::stoi(argv[++i]));
        } else if (a == "--tlm-div" && has_val) {
            opt.tlm_divider = std::max(1, std::stoi(argv[++i]));
        } else if (a == "--exit-after" && has_val) {
            opt.exit_after_terminal_s = std::stod(argv[++i]);
        } else if (a == "--max-time" && has_val) {
            opt.max_time_s = std::stod(argv[++i]);
        } else {
            return usage();
        }
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    fc::net::UdpSocket sensor_rx = fc::net::UdpSocket::bound("127.0.0.1", opt.sensor_port);
    fc::net::UdpSocket cmd_sock = fc::net::UdpSocket::bound("0.0.0.0", opt.cmd_port);  // ACKs go out from here too
    fc::net::UdpSocket tx;      // actuators to the sim
    fc::net::UdpSocket tlm_tx;  // telemetry to the ground
    // The ground links model a radio: integrity comes from the packet CRC-32, not UDP.
    cmd_sock.disable_udp_checksum();
    tlm_tx.disable_udp_checksum();
    const sockaddr_in sim_addr = fc::net::parse_endpoint(opt.sim_endpoint);
    const sockaddr_in gs_addr = fc::net::parse_endpoint(opt.gs_endpoint);

    fc::rt::minimize_timer_slack();
    bool rt_enabled = false;
    if (opt.realtime) {
        std::string why;
        rt_enabled = fc::rt::enable_realtime(80, why);
        std::printf("[fc] SCHED_FIFO %s%s\n", rt_enabled ? "enabled" : "unavailable: ", why.c_str());
    }

    const fc::FcConfig cfg;
    fc::FlightComputer computer(cfg);
    fc::flightlog::Writer log;
    if (opt.log_path != "none" && !log.open(opt.log_path, cfg))
        std::fprintf(stderr, "[fc] cannot open log %s\n", opt.log_path.c_str());

    // Preallocated so the loop never allocates for timing data.
    const auto max_frames = static_cast<std::size_t>(opt.max_time_s * 100.0) + 1;
    std::vector<double> wake_us;
    std::vector<double> jitter_us;
    wake_us.reserve(max_frames);
    jitter_us.reserve(max_frames);

    std::array<std::uint8_t, fc::proto::kMaxPacketSize> buf{};
    std::array<fc::proto::SensorSample, fc::kMaxSensorsPerTick> sensors{};
    std::array<fc::proto::Command, fc::kMaxCommandsPerTick> cmds{};
    std::array<sockaddr_in, fc::kMaxCommandsPerTick> cmd_src{};
    std::uint32_t tlm_seq = 0;
    std::uint32_t act_seq = 0;
    std::uint32_t ack_seq = 0;
    std::uint32_t crc_errors = 0;
    float rolling_p99 = 0.0f;
    float max_wake = 0.0f;
    std::uint32_t terminal_frames = 0;
    fc::proto::Telemetry last_tlm;

    std::printf("[fc] 100 Hz loop: sensors :%u, uplink :%u, sim %s, gs %s, log %s\n", opt.sensor_port, opt.cmd_port,
                opt.sim_endpoint.c_str(), opt.gs_endpoint.c_str(), opt.log_path.c_str());
    std::fflush(stdout);

    fc::rt::PeriodicTimer timer(kPeriodNs, std::int64_t{opt.spin_us} * 1000);
    std::int64_t last_wake = 0;
    fc::FlightState last_state = fc::FlightState::Pad;

    while (!g_stop && wake_us.size() < max_frames) {
        const std::int64_t late_ns = timer.wait();
        const std::int64_t wake = fc::rt::monotonic_ns();
        wake_us.push_back(static_cast<double>(late_ns) / 1e3);
        if (last_wake != 0) jitter_us.push_back(std::abs(static_cast<double>(wake - last_wake - kPeriodNs)) / 1e3);
        last_wake = wake;
        max_wake = std::max(max_wake, static_cast<float>(wake_us.back()));

        std::size_t n_sensors = 0;
        while (n_sensors < sensors.size()) {
            const auto n = sensor_rx.recv(buf);
            if (!n) break;
            fc::proto::Header h;
            if (fc::proto::decode_as(std::span<const std::uint8_t>(buf.data(), *n), h, sensors[n_sensors]) ==
                fc::proto::DecodeError::None)
                ++n_sensors;
        }

        std::size_t n_cmds = 0;
        while (n_cmds < cmds.size()) {
            sockaddr_in from{};
            const auto n = cmd_sock.recv(buf, &from);
            if (!n) break;
            fc::proto::Header h;
            if (fc::proto::decode_as(std::span<const std::uint8_t>(buf.data(), *n), h, cmds[n_cmds]) ==
                fc::proto::DecodeError::None) {
                cmd_src[n_cmds++] = from;
            } else {
                ++crc_errors;
            }
        }

        const fc::FcInputs in{{sensors.data(), n_sensors}, {cmds.data(), n_cmds}};
        const fc::FcOutputs& out = computer.step(in);
        const auto now_us = static_cast<std::uint64_t>(wake / 1000);

        std::size_t len = fc::proto::encode(out.actuator, act_seq++, now_us, buf);
        tx.send_to({buf.data(), len}, sim_addr);

        for (std::size_t i = 0; i < out.n_acks; ++i) {
            len = fc::proto::encode(out.acks[i], ack_seq++, now_us, buf);
            cmd_sock.send_to({buf.data(), len}, cmd_src[i]);
        }

        if (wake_us.size() % kRollingWindow == 0) {
            rolling_p99 = static_cast<float>(
                fc::summarize(std::span<const double>(wake_us).last(kRollingWindow)).p99);
        }
        if (computer.tick() % static_cast<std::uint32_t>(opt.tlm_divider) == 0) {
            fc::proto::Telemetry t = out.telemetry;
            t.uplink_crc_errors = crc_errors;
            t.loop_overruns = timer.overruns();
            t.loop_p99_us = rolling_p99;
            t.loop_max_us = max_wake;
            len = fc::proto::encode(t, tlm_seq++, now_us, buf);
            tlm_tx.send_to({buf.data(), len}, gs_addr);
        }

        log.write(computer.tick(), in, out);
        last_tlm = out.telemetry;

        const fc::FlightState st = computer.state();
        if (st != last_state) {
            std::printf("[fc] t=%.2f %s -> %s  (h=%.1f v=%.1f faults=0x%04x)\n", computer.tick() * 0.01,
                        fc::to_string(last_state), fc::to_string(st), computer.filter().h(), computer.filter().v(),
                        computer.faults());
            std::fflush(stdout);
            last_state = st;
        }
        const bool terminal = st == fc::FlightState::Landed || st == fc::FlightState::Abort;
        terminal_frames = terminal ? terminal_frames + 1 : 0;
        if (terminal && terminal_frames >= static_cast<std::uint32_t>(opt.exit_after_terminal_s * 100.0)) break;
    }
    log.close();

    const fc::LatencySummary wake = fc::summarize(wake_us);
    const fc::LatencySummary jitter = fc::summarize(jitter_us);
    std::printf("[fc] final state %s (abort reason: %s), %zu frames, %u overruns, %u uplink decode errors\n",
                fc::to_string(computer.state()), fc::to_string(computer.state_machine().abort_reason()), wake.count,
                timer.overruns(), crc_errors);
    std::printf("[fc] wake latency  us: p50 %.1f  p99 %.1f  p99.9 %.1f  max %.1f\n", wake.p50, wake.p99, wake.p999,
                wake.max);
    std::printf("[fc] period jitter us: p50 %.1f  p99 %.1f  p99.9 %.1f  max %.1f\n", jitter.p50, jitter.p99,
                jitter.p999, jitter.max);
    if (!opt.stats_path.empty())
        write_stats(opt.stats_path, wake, jitter, timer.overruns(), crc_errors, computer, last_tlm, rt_enabled,
                    opt.spin_us);
    return 0;
}
