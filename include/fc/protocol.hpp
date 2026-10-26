#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

namespace fc::proto {

// Wire format, all fields little-endian:
//
//   magic u16 | version u8 | type u8 | seq u32 | timestamp_us u64 | payload_len u16
//   payload[payload_len]
//   crc32 u32   (CRC-32 over header + payload)
//
// ground/protocol.py mirrors this layout; keep the two in sync.
inline constexpr std::uint16_t kMagic = 0xFC5A;
inline constexpr std::uint8_t kVersion = 1;
inline constexpr std::size_t kHeaderSize = 18;
inline constexpr std::size_t kCrcSize = 4;
inline constexpr std::size_t kMaxPacketSize = 256;

enum class MsgType : std::uint8_t { Sensor = 1, Actuator = 2, Telemetry = 3, Command = 4, Ack = 5 };

struct Header {
    MsgType type{};
    std::uint32_t seq = 0;
    std::uint64_t timestamp_us = 0;
    std::uint16_t payload_len = 0;
};

inline constexpr std::uint8_t kSensorImuValid = 1u << 0;
inline constexpr std::uint8_t kSensorAltValid = 1u << 1;

// Sim -> flight computer, 100 Hz.
struct SensorSample {
    std::uint64_t sim_time_us = 0;
    float accel_mps2 = 0.0f;  // specific force along the vehicle axis (reads +g on the pad)
    float alt_m = 0.0f;
    std::uint8_t flags = 0;

    bool imu_valid() const { return (flags & kSensorImuValid) != 0; }
    bool alt_valid() const { return (flags & kSensorAltValid) != 0; }
};

// Flight computer -> sim, 100 Hz.
struct ActuatorCmd {
    float throttle = 0.0f;
    std::uint8_t engine_on = 0;
    std::uint8_t chute = 0;
};

enum class Opcode : std::uint8_t { Arm = 1, Abort = 2, SetParam = 3, Ping = 4 };
enum class ParamId : std::uint8_t {
    AscentBurnS = 1,
    LandingThrottle = 2,
    LandingKp = 3,
    TerminalVelocity = 4,
    IgnitionMarginM = 5,
};
enum class AckResult : std::uint8_t { Ok = 0, Rejected = 1, BadParam = 2, BadOpcode = 3 };

// Ground -> flight computer. `session` is chosen randomly by each ground-station
// run; `cmd_id` increases by one per new command within a session.
struct Command {
    std::uint32_t session = 0;
    std::uint32_t cmd_id = 0;
    Opcode opcode = Opcode::Ping;
    std::uint8_t param = 0;
    float value = 0.0f;
};

// Flight computer -> ground, sent back to the command's source address.
struct Ack {
    std::uint32_t session = 0;
    std::uint32_t cmd_id = 0;
    Opcode opcode = Opcode::Ping;
    AckResult result = AckResult::Ok;
};

// Flight computer -> ground.
struct Telemetry {
    std::uint32_t tick = 0;
    std::uint8_t state = 0;
    std::uint16_t faults = 0;
    float h_est = 0.0f;
    float v_est = 0.0f;
    float bias_est = 0.0f;
    float h_sigma = 0.0f;
    float accel_meas = 0.0f;
    float alt_meas = 0.0f;
    std::uint8_t sensor_flags = 0;
    float throttle_cmd = 0.0f;
    float mass_est = 0.0f;
    float ignition_alt = 0.0f;
    std::uint32_t cmds_accepted = 0;
    std::uint32_t cmds_duplicate = 0;
    std::uint32_t alt_rejects = 0;
    // Filled in by the real-time executive, not by the deterministic core.
    std::uint32_t uplink_crc_errors = 0;
    std::uint32_t loop_overruns = 0;
    float loop_p99_us = 0.0f;
    float loop_max_us = 0.0f;
};

inline constexpr std::size_t kSensorPayloadSize = 17;
inline constexpr std::size_t kActuatorPayloadSize = 6;
inline constexpr std::size_t kCommandPayloadSize = 14;
inline constexpr std::size_t kAckPayloadSize = 10;
inline constexpr std::size_t kTelemetryPayloadSize = 72;

constexpr MsgType msg_type(const SensorSample&) { return MsgType::Sensor; }
constexpr MsgType msg_type(const ActuatorCmd&) { return MsgType::Actuator; }
constexpr MsgType msg_type(const Telemetry&) { return MsgType::Telemetry; }
constexpr MsgType msg_type(const Command&) { return MsgType::Command; }
constexpr MsgType msg_type(const Ack&) { return MsgType::Ack; }

enum class DecodeError : std::uint8_t {
    None = 0,
    TooShort,
    BadMagic,
    BadVersion,
    BadLength,
    BadCrc,
    BadType,
    BadPayload,
};
const char* to_string(DecodeError err);

// Bounds-checked little-endian writer over a caller-owned buffer. Never allocates;
// on overflow it stops writing and ok() turns false.
class ByteWriter {
public:
    explicit ByteWriter(std::span<std::uint8_t> buf) : buf_(buf) {}

    void u8(std::uint8_t v) { put(v, 1); }
    void u16(std::uint16_t v) { put(v, 2); }
    void u32(std::uint32_t v) { put(v, 4); }
    void u64(std::uint64_t v) { put(v, 8); }
    void f32(float v) { put(std::bit_cast<std::uint32_t>(v), 4); }
    void f64(double v) { put(std::bit_cast<std::uint64_t>(v), 8); }

    std::size_t size() const { return pos_; }
    bool ok() const { return ok_; }

private:
    void put(std::uint64_t v, std::size_t n);

    std::span<std::uint8_t> buf_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

// Bounds-checked little-endian reader. Reads past the end return 0 and clear ok().
class ByteReader {
public:
    explicit ByteReader(std::span<const std::uint8_t> buf) : buf_(buf) {}

    std::uint8_t u8() { return static_cast<std::uint8_t>(get(1)); }
    std::uint16_t u16() { return static_cast<std::uint16_t>(get(2)); }
    std::uint32_t u32() { return static_cast<std::uint32_t>(get(4)); }
    std::uint64_t u64() { return get(8); }
    float f32() { return std::bit_cast<float>(u32()); }
    double f64() { return std::bit_cast<double>(u64()); }

    std::size_t remaining() const { return buf_.size() - pos_; }
    bool ok() const { return ok_; }

private:
    std::uint64_t get(std::size_t n);

    std::span<const std::uint8_t> buf_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

// Payload codecs (no framing). Also used by the flight log.
void write(ByteWriter& w, const SensorSample& m);
void write(ByteWriter& w, const ActuatorCmd& m);
void write(ByteWriter& w, const Telemetry& m);
void write(ByteWriter& w, const Command& m);
void write(ByteWriter& w, const Ack& m);
void read(ByteReader& r, SensorSample& m);
void read(ByteReader& r, ActuatorCmd& m);
void read(ByteReader& r, Telemetry& m);
void read(ByteReader& r, Command& m);
void read(ByteReader& r, Ack& m);

// Frames `msg` into `out`. Returns the packet length, or 0 if `out` is too small.
std::size_t encode(const SensorSample& msg, std::uint32_t seq, std::uint64_t ts_us, std::span<std::uint8_t> out);
std::size_t encode(const ActuatorCmd& msg, std::uint32_t seq, std::uint64_t ts_us, std::span<std::uint8_t> out);
std::size_t encode(const Telemetry& msg, std::uint32_t seq, std::uint64_t ts_us, std::span<std::uint8_t> out);
std::size_t encode(const Command& msg, std::uint32_t seq, std::uint64_t ts_us, std::span<std::uint8_t> out);
std::size_t encode(const Ack& msg, std::uint32_t seq, std::uint64_t ts_us, std::span<std::uint8_t> out);

// Validates framing and CRC. On success fills `header` and points `payload` into `bytes`.
DecodeError decode(std::span<const std::uint8_t> bytes, Header& header, std::span<const std::uint8_t>& payload);

// Parses a payload that must be exactly the size of the message.
template <class T>
bool parse(std::span<const std::uint8_t> payload, T& msg) {
    ByteReader r(payload);
    read(r, msg);
    return r.ok() && r.remaining() == 0;
}

template <class T>
DecodeError decode_as(std::span<const std::uint8_t> bytes, Header& header, T& msg) {
    std::span<const std::uint8_t> payload;
    if (const DecodeError err = decode(bytes, header, payload); err != DecodeError::None) return err;
    if (header.type != msg_type(msg)) return DecodeError::BadType;
    return parse(payload, msg) ? DecodeError::None : DecodeError::BadPayload;
}

}  // namespace fc::proto
