#include "fc/protocol.hpp"

#include "fc/crc32.hpp"

namespace fc::proto {

void ByteWriter::put(std::uint64_t v, std::size_t n) {
    if (!ok_ || pos_ + n > buf_.size()) {
        ok_ = false;
        return;
    }
    for (std::size_t i = 0; i < n; ++i) buf_[pos_++] = static_cast<std::uint8_t>(v >> (8 * i));
}

std::uint64_t ByteReader::get(std::size_t n) {
    if (!ok_ || pos_ + n > buf_.size()) {
        ok_ = false;
        return 0;
    }
    std::uint64_t v = 0;
    for (std::size_t i = 0; i < n; ++i) v |= static_cast<std::uint64_t>(buf_[pos_++]) << (8 * i);
    return v;
}

const char* to_string(DecodeError err) {
    switch (err) {
        case DecodeError::None: return "none";
        case DecodeError::TooShort: return "too_short";
        case DecodeError::BadMagic: return "bad_magic";
        case DecodeError::BadVersion: return "bad_version";
        case DecodeError::BadLength: return "bad_length";
        case DecodeError::BadCrc: return "bad_crc";
        case DecodeError::BadType: return "bad_type";
        case DecodeError::BadPayload: return "bad_payload";
    }
    return "unknown";
}

void write(ByteWriter& w, const SensorSample& m) {
    w.u64(m.sim_time_us);
    w.f32(m.accel_mps2);
    w.f32(m.alt_m);
    w.u8(m.flags);
}

void read(ByteReader& r, SensorSample& m) {
    m.sim_time_us = r.u64();
    m.accel_mps2 = r.f32();
    m.alt_m = r.f32();
    m.flags = r.u8();
}

void write(ByteWriter& w, const ActuatorCmd& m) {
    w.f32(m.throttle);
    w.u8(m.engine_on);
    w.u8(m.chute);
}

void read(ByteReader& r, ActuatorCmd& m) {
    m.throttle = r.f32();
    m.engine_on = r.u8();
    m.chute = r.u8();
}

void write(ByteWriter& w, const Telemetry& m) {
    w.u32(m.tick);
    w.u8(m.state);
    w.u16(m.faults);
    w.f32(m.h_est);
    w.f32(m.v_est);
    w.f32(m.bias_est);
    w.f32(m.h_sigma);
    w.f32(m.accel_meas);
    w.f32(m.alt_meas);
    w.u8(m.sensor_flags);
    w.f32(m.throttle_cmd);
    w.f32(m.mass_est);
    w.f32(m.ignition_alt);
    w.u32(m.cmds_accepted);
    w.u32(m.cmds_duplicate);
    w.u32(m.alt_rejects);
    w.u32(m.uplink_crc_errors);
    w.u32(m.loop_overruns);
    w.f32(m.loop_p99_us);
    w.f32(m.loop_max_us);
}

void read(ByteReader& r, Telemetry& m) {
    m.tick = r.u32();
    m.state = r.u8();
    m.faults = r.u16();
    m.h_est = r.f32();
    m.v_est = r.f32();
    m.bias_est = r.f32();
    m.h_sigma = r.f32();
    m.accel_meas = r.f32();
    m.alt_meas = r.f32();
    m.sensor_flags = r.u8();
    m.throttle_cmd = r.f32();
    m.mass_est = r.f32();
    m.ignition_alt = r.f32();
    m.cmds_accepted = r.u32();
    m.cmds_duplicate = r.u32();
    m.alt_rejects = r.u32();
    m.uplink_crc_errors = r.u32();
    m.loop_overruns = r.u32();
    m.loop_p99_us = r.f32();
    m.loop_max_us = r.f32();
}

void write(ByteWriter& w, const Command& m) {
    w.u32(m.session);
    w.u32(m.cmd_id);
    w.u8(static_cast<std::uint8_t>(m.opcode));
    w.u8(m.param);
    w.f32(m.value);
}

void read(ByteReader& r, Command& m) {
    m.session = r.u32();
    m.cmd_id = r.u32();
    m.opcode = static_cast<Opcode>(r.u8());
    m.param = r.u8();
    m.value = r.f32();
}

void write(ByteWriter& w, const Ack& m) {
    w.u32(m.session);
    w.u32(m.cmd_id);
    w.u8(static_cast<std::uint8_t>(m.opcode));
    w.u8(static_cast<std::uint8_t>(m.result));
}

void read(ByteReader& r, Ack& m) {
    m.session = r.u32();
    m.cmd_id = r.u32();
    m.opcode = static_cast<Opcode>(r.u8());
    m.result = static_cast<AckResult>(r.u8());
}

namespace {

template <class T>
std::size_t encode_impl(const T& msg, std::uint32_t seq, std::uint64_t ts_us, std::span<std::uint8_t> out) {
    if (out.size() < kHeaderSize + kCrcSize) return 0;
    ByteWriter body(out.subspan(kHeaderSize, out.size() - kHeaderSize - kCrcSize));
    write(body, msg);
    if (!body.ok()) return 0;

    const std::size_t payload_len = body.size();
    ByteWriter header(out.first(kHeaderSize));
    header.u16(kMagic);
    header.u8(kVersion);
    header.u8(static_cast<std::uint8_t>(msg_type(msg)));
    header.u32(seq);
    header.u64(ts_us);
    header.u16(static_cast<std::uint16_t>(payload_len));

    const std::size_t crc_pos = kHeaderSize + payload_len;
    ByteWriter trailer(out.subspan(crc_pos, kCrcSize));
    trailer.u32(crc32(out.first(crc_pos)));
    return crc_pos + kCrcSize;
}

bool known_type(std::uint8_t t) {
    return t >= static_cast<std::uint8_t>(MsgType::Sensor) && t <= static_cast<std::uint8_t>(MsgType::Ack);
}

}  // namespace

std::size_t encode(const SensorSample& m, std::uint32_t seq, std::uint64_t ts, std::span<std::uint8_t> out) {
    return encode_impl(m, seq, ts, out);
}
std::size_t encode(const ActuatorCmd& m, std::uint32_t seq, std::uint64_t ts, std::span<std::uint8_t> out) {
    return encode_impl(m, seq, ts, out);
}
std::size_t encode(const Telemetry& m, std::uint32_t seq, std::uint64_t ts, std::span<std::uint8_t> out) {
    return encode_impl(m, seq, ts, out);
}
std::size_t encode(const Command& m, std::uint32_t seq, std::uint64_t ts, std::span<std::uint8_t> out) {
    return encode_impl(m, seq, ts, out);
}
std::size_t encode(const Ack& m, std::uint32_t seq, std::uint64_t ts, std::span<std::uint8_t> out) {
    return encode_impl(m, seq, ts, out);
}

DecodeError decode(std::span<const std::uint8_t> bytes, Header& header, std::span<const std::uint8_t>& payload) {
    if (bytes.size() < kHeaderSize + kCrcSize) return DecodeError::TooShort;

    ByteReader r(bytes.first(kHeaderSize));
    if (r.u16() != kMagic) return DecodeError::BadMagic;
    if (r.u8() != kVersion) return DecodeError::BadVersion;
    const std::uint8_t type = r.u8();
    header.seq = r.u32();
    header.timestamp_us = r.u64();
    header.payload_len = r.u16();

    const std::size_t body_len = kHeaderSize + header.payload_len;
    if (body_len + kCrcSize != bytes.size()) return DecodeError::BadLength;

    ByteReader trailer(bytes.subspan(body_len, kCrcSize));
    if (trailer.u32() != crc32(bytes.first(body_len))) return DecodeError::BadCrc;
    if (!known_type(type)) return DecodeError::BadType;

    header.type = static_cast<MsgType>(type);
    payload = bytes.subspan(kHeaderSize, header.payload_len);
    return DecodeError::None;
}

}  // namespace fc::proto
