#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "fc/protocol.hpp"

namespace {

using namespace fc::proto;

using Buffer = std::array<std::uint8_t, kMaxPacketSize>;

template <class T>
std::vector<std::uint8_t> frame(const T& msg, std::uint32_t seq = 7, std::uint64_t ts = 123456789) {
    Buffer buf{};
    const std::size_t n = encode(msg, seq, ts, buf);
    return {buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(n)};
}

Telemetry sample_telemetry() {
    Telemetry t;
    t.tick = 4242;
    t.state = 4;
    t.faults = 0x0003;
    t.h_est = 123.25f;
    t.v_est = -45.5f;
    t.bias_est = 0.04f;
    t.h_sigma = 0.3f;
    t.accel_meas = 15.0f;
    t.alt_meas = 123.0f;
    t.sensor_flags = kSensorImuValid | kSensorAltValid;
    t.throttle_cmd = 0.62f;
    t.mass_est = 912.5f;
    t.ignition_alt = 300.0f;
    t.cmds_accepted = 3;
    t.cmds_duplicate = 9;
    t.alt_rejects = 1;
    t.uplink_crc_errors = 2;
    t.loop_overruns = 0;
    t.loop_p99_us = 87.5f;
    t.loop_max_us = 412.0f;
    return t;
}

TEST(Protocol, SensorRoundTrip) {
    SensorSample in{1'000'000, 9.81f, 12.5f, kSensorImuValid | kSensorAltValid};
    const auto pkt = frame(in, 11, 1'000'000);
    ASSERT_EQ(pkt.size(), kHeaderSize + kSensorPayloadSize + kCrcSize);
    Header h;
    SensorSample out;
    ASSERT_EQ(decode_as(pkt, h, out), DecodeError::None);
    EXPECT_EQ(h.type, MsgType::Sensor);
    EXPECT_EQ(h.seq, 11u);
    EXPECT_EQ(h.timestamp_us, 1'000'000u);
    EXPECT_EQ(out.sim_time_us, in.sim_time_us);
    EXPECT_EQ(out.accel_mps2, in.accel_mps2);
    EXPECT_EQ(out.alt_m, in.alt_m);
    EXPECT_TRUE(out.imu_valid());
    EXPECT_TRUE(out.alt_valid());
}

TEST(Protocol, ActuatorRoundTrip) {
    const ActuatorCmd in{0.55f, 1, 0};
    Header h;
    ActuatorCmd out;
    ASSERT_EQ(decode_as(frame(in), h, out), DecodeError::None);
    EXPECT_EQ(out.throttle, 0.55f);
    EXPECT_EQ(out.engine_on, 1);
    EXPECT_EQ(out.chute, 0);
}

TEST(Protocol, CommandAndAckRoundTrip) {
    const Command cmd{0xDEADBEEF, 42, Opcode::SetParam, static_cast<std::uint8_t>(ParamId::LandingKp), 1.25f};
    Header h;
    Command c;
    ASSERT_EQ(decode_as(frame(cmd), h, c), DecodeError::None);
    EXPECT_EQ(c.session, 0xDEADBEEFu);
    EXPECT_EQ(c.cmd_id, 42u);
    EXPECT_EQ(c.opcode, Opcode::SetParam);
    EXPECT_EQ(c.param, static_cast<std::uint8_t>(ParamId::LandingKp));
    EXPECT_EQ(c.value, 1.25f);

    const Ack ack{0xDEADBEEF, 42, Opcode::SetParam, AckResult::BadParam};
    Ack a;
    ASSERT_EQ(decode_as(frame(ack), h, a), DecodeError::None);
    EXPECT_EQ(a.cmd_id, 42u);
    EXPECT_EQ(a.result, AckResult::BadParam);
}

TEST(Protocol, TelemetryRoundTripPreservesEveryField) {
    const Telemetry in = sample_telemetry();
    const auto pkt = frame(in);
    ASSERT_EQ(pkt.size(), kHeaderSize + kTelemetryPayloadSize + kCrcSize);
    Header h;
    Telemetry out;
    ASSERT_EQ(decode_as(pkt, h, out), DecodeError::None);
    EXPECT_EQ(out.tick, in.tick);
    EXPECT_EQ(out.state, in.state);
    EXPECT_EQ(out.faults, in.faults);
    EXPECT_EQ(out.h_est, in.h_est);
    EXPECT_EQ(out.v_est, in.v_est);
    EXPECT_EQ(out.mass_est, in.mass_est);
    EXPECT_EQ(out.ignition_alt, in.ignition_alt);
    EXPECT_EQ(out.cmds_duplicate, in.cmds_duplicate);
    EXPECT_EQ(out.loop_p99_us, in.loop_p99_us);
    EXPECT_EQ(out.loop_max_us, in.loop_max_us);
}

TEST(Protocol, HeaderIsLittleEndianOnTheWire) {
    const auto pkt = frame(ActuatorCmd{}, 0x01020304, 0x1122334455667788ull);
    EXPECT_EQ(pkt[0], 0x5A);  // magic 0xFC5A, LSB first
    EXPECT_EQ(pkt[1], 0xFC);
    EXPECT_EQ(pkt[2], kVersion);
    EXPECT_EQ(pkt[3], static_cast<std::uint8_t>(MsgType::Actuator));
    EXPECT_EQ(pkt[4], 0x04);
    EXPECT_EQ(pkt[7], 0x01);
    EXPECT_EQ(pkt[8], 0x88);
    EXPECT_EQ(pkt[15], 0x11);
    EXPECT_EQ(pkt[16], kActuatorPayloadSize);
    EXPECT_EQ(pkt[17], 0);
}

TEST(Protocol, EverySingleBitFlipIsRejected) {
    const auto good = frame(sample_telemetry());
    for (std::size_t byte = 0; byte < good.size(); ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            auto bad = good;
            bad[byte] ^= static_cast<std::uint8_t>(1u << bit);
            Header h;
            Telemetry t;
            EXPECT_NE(decode_as(bad, h, t), DecodeError::None) << "byte " << byte << " bit " << bit;
        }
    }
}

TEST(Protocol, PayloadCorruptionReportsBadCrc) {
    auto pkt = frame(sample_telemetry());
    pkt[kHeaderSize + 5] ^= 0xFF;
    Header h;
    Telemetry t;
    EXPECT_EQ(decode_as(pkt, h, t), DecodeError::BadCrc);
}

TEST(Protocol, TruncatedAndPaddedPacketsAreRejected) {
    const auto good = frame(sample_telemetry());
    Header h;
    Telemetry t;
    for (std::size_t len = 0; len < good.size(); ++len) {
        const std::vector<std::uint8_t> cut(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(len));
        const DecodeError e = decode_as(cut, h, t);
        EXPECT_TRUE(e == DecodeError::TooShort || e == DecodeError::BadLength) << "len " << len;
    }
    auto padded = good;
    padded.push_back(0);
    EXPECT_EQ(decode_as(padded, h, t), DecodeError::BadLength);
}

TEST(Protocol, BadMagicAndVersionAreDistinguished) {
    auto pkt = frame(ActuatorCmd{});
    Header h;
    ActuatorCmd a;
    auto bad_magic = pkt;
    bad_magic[0] = 0x00;
    EXPECT_EQ(decode_as(bad_magic, h, a), DecodeError::BadMagic);
    auto bad_version = pkt;
    bad_version[2] = 99;
    EXPECT_EQ(decode_as(bad_version, h, a), DecodeError::BadVersion);
}

TEST(Protocol, WrongMessageTypeIsRejected) {
    const auto pkt = frame(ActuatorCmd{});
    Header h;
    Telemetry t;
    EXPECT_EQ(decode_as(pkt, h, t), DecodeError::BadType);
}

TEST(Protocol, EncodeFailsCleanlyWhenBufferTooSmall) {
    std::array<std::uint8_t, 40> small{};
    EXPECT_EQ(encode(sample_telemetry(), 0, 0, small), 0u);
    std::array<std::uint8_t, kHeaderSize + kActuatorPayloadSize + kCrcSize> exact{};
    EXPECT_EQ(encode(ActuatorCmd{}, 0, 0, exact), exact.size());
}

TEST(Protocol, ReaderFlagsUnderrun) {
    const std::array<std::uint8_t, 3> data{1, 2, 3};
    ByteReader r(data);
    EXPECT_EQ(r.u16(), 0x0201);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.u32(), 0u);
    EXPECT_FALSE(r.ok());
}

}  // namespace
