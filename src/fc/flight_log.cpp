#include "fc/flight_log.hpp"

#include <array>
#include <cstring>
#include <type_traits>

namespace fc::flightlog {
namespace {

constexpr char kMagic[8] = {'F', 'C', 'L', 'O', 'G', '0', '1', '\0'};
constexpr std::size_t kMaxRecord = 4096;

// Single list of persisted config fields, shared by the writer and the reader so
// the two cannot drift apart.
template <class Cfg, class F>
void for_each_field(Cfg& c, F&& f) {
    f(c.vehicle.dry_mass_kg);
    f(c.vehicle.prop_mass_kg);
    f(c.vehicle.max_thrust_n);
    f(c.vehicle.min_throttle);
    f(c.vehicle.isp_s);
    f(c.vehicle.engine_tau_s);
    f(c.vehicle.drag_cd);
    f(c.vehicle.ref_area_m2);
    f(c.vehicle.chute_cd_area_m2);
    f(c.kf.accel_sigma);
    f(c.kf.bias_walk_sigma);
    f(c.kf.alt_sigma);
    f(c.kf.gate_sigma);
    f(c.kf.init_h_sigma);
    f(c.kf.init_v_sigma);
    f(c.kf.init_b_sigma);
    f(c.guidance.plan_throttle);
    f(c.guidance.ignition_margin_m);
    f(c.guidance.kp);
    f(c.guidance.terminal_velocity);
    f(c.guidance.terminal_alt_m);
    f(c.sm.dt);
    f(c.sm.countdown_s);
    f(c.sm.ascent_burn_s);
    f(c.sm.landing_reserve_kg);
    f(c.sm.apogee_confirm_ticks);
    f(c.sm.landed_h);
    f(c.sm.landed_v);
    f(c.sm.landed_confirm_ticks);
    f(c.faults.imu_stuck_samples);
    f(c.faults.imu_clear_samples);
    f(c.faults.alt_stale_ticks);
    f(c.faults.sensor_link_ticks);
    f(c.faults.alt_outlier_hold_ticks);
    f(c.faults.alt_force_accept);
    f(c.faults.max_h_sigma);
    f(c.model_accel_sigma);
    f(c.zupt_alt_m);
    f(c.zupt_sigma);
    f(c.sensor_dt_s);
}

void write_u32(std::FILE* f, std::uint32_t v) {
    std::array<std::uint8_t, 4> b{};
    proto::ByteWriter w(b);
    w.u32(v);
    std::fwrite(b.data(), 1, b.size(), f);
}

bool read_u32(std::FILE* f, std::uint32_t& v) {
    std::array<std::uint8_t, 4> b{};
    if (std::fread(b.data(), 1, b.size(), f) != b.size()) return false;
    proto::ByteReader r(b);
    v = r.u32();
    return true;
}

}  // namespace

Writer::~Writer() { close(); }

bool Writer::open(const std::string& path, const FcConfig& cfg) {
    close();
    file_ = std::fopen(path.c_str(), "wb");
    if (file_ == nullptr) return false;
    std::setvbuf(file_, nullptr, _IOFBF, 1 << 16);

    std::array<std::uint8_t, 1024> cbuf{};
    proto::ByteWriter w(cbuf);
    for_each_field(cfg, [&w](const auto& v) {
        if constexpr (std::is_same_v<std::decay_t<decltype(v)>, double>) {
            w.f64(v);
        } else {
            w.u32(static_cast<std::uint32_t>(v));
        }
    });
    std::fwrite(kMagic, 1, sizeof kMagic, file_);
    write_u32(file_, static_cast<std::uint32_t>(w.size()));
    std::fwrite(cbuf.data(), 1, w.size(), file_);
    return w.ok();
}

void Writer::write(std::uint32_t tick, const FcInputs& in, const FcOutputs& out) {
    if (file_ == nullptr) return;
    std::array<std::uint8_t, kMaxRecord> buf{};
    proto::ByteWriter w(buf);
    w.u32(tick);
    w.u8(static_cast<std::uint8_t>(in.sensors.size()));
    for (const auto& s : in.sensors) proto::write(w, s);
    const std::size_t n_cmd = std::min(in.commands.size(), kMaxCommandsPerTick);
    w.u8(static_cast<std::uint8_t>(n_cmd));
    for (const auto& c : in.commands.first(n_cmd)) proto::write(w, c);
    proto::write(w, out.actuator);
    proto::write(w, out.telemetry);
    w.u8(static_cast<std::uint8_t>(out.n_acks));
    for (const auto& a : out.ack_span()) proto::write(w, a);
    if (!w.ok()) return;
    write_u32(file_, static_cast<std::uint32_t>(w.size()));
    std::fwrite(buf.data(), 1, w.size(), file_);
}

void Writer::close() {
    if (file_ != nullptr) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

Reader::~Reader() {
    if (file_ != nullptr) std::fclose(file_);
}

bool Reader::open(const std::string& path, FcConfig& cfg) {
    file_ = std::fopen(path.c_str(), "rb");
    if (file_ == nullptr) {
        error_ = "cannot open " + path;
        return false;
    }
    char magic[sizeof kMagic] = {};
    if (std::fread(magic, 1, sizeof magic, file_) != sizeof magic || std::memcmp(magic, kMagic, sizeof magic) != 0) {
        error_ = "not a flight log (bad magic)";
        return false;
    }
    std::uint32_t len = 0;
    if (!read_u32(file_, len) || len > kMaxRecord) {
        error_ = "bad config header";
        return false;
    }
    buf_.resize(len);
    if (std::fread(buf_.data(), 1, len, file_) != len) {
        error_ = "truncated config";
        return false;
    }
    proto::ByteReader r(buf_);
    for_each_field(cfg, [&r](auto& v) {
        if constexpr (std::is_same_v<std::decay_t<decltype(v)>, double>) {
            v = r.f64();
        } else {
            v = static_cast<int>(r.u32());
        }
    });
    if (!r.ok() || r.remaining() != 0) {
        error_ = "config layout mismatch (log written by a different version?)";
        return false;
    }
    return true;
}

bool Reader::next(TickRecord& rec) {
    if (file_ == nullptr) return false;
    std::uint32_t len = 0;
    if (!read_u32(file_, len)) return false;  // clean end of file
    if (len > kMaxRecord) {
        error_ = "record too large";
        return false;
    }
    buf_.resize(len);
    if (std::fread(buf_.data(), 1, len, file_) != len) {
        error_ = "truncated record";
        return false;
    }
    proto::ByteReader r(buf_);
    rec.tick = r.u32();
    rec.sensors.resize(r.u8());
    for (auto& s : rec.sensors) proto::read(r, s);
    rec.commands.resize(r.u8());
    for (auto& c : rec.commands) proto::read(r, c);
    proto::read(r, rec.actuator);
    proto::read(r, rec.telemetry);
    rec.acks.resize(r.u8());
    for (auto& a : rec.acks) proto::read(r, a);
    if (!r.ok() || r.remaining() != 0) {
        error_ = "malformed record at tick " + std::to_string(rec.tick);
        return false;
    }
    return true;
}

}  // namespace fc::flightlog
