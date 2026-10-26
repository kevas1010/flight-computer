#include <gtest/gtest.h>

#include <vector>

#include "fc/command_receiver.hpp"
#include "fc/flight_computer.hpp"

namespace {

using fc::proto::Ack;
using fc::proto::AckResult;
using fc::proto::Command;
using fc::proto::Opcode;
using fc::proto::ParamId;

TEST(CommandReceiver, NewCommandIsExecuted) {
    fc::CommandReceiver rx;
    EXPECT_FALSE(rx.duplicate_of({1, 1, Opcode::Ping, 0, 0}).has_value());
}

TEST(CommandReceiver, RetransmissionGetsOriginalAckWithoutReexecution) {
    fc::CommandReceiver rx;
    const Command cmd{1, 1, Opcode::Arm, 0, 0};
    ASSERT_FALSE(rx.duplicate_of(cmd));
    rx.record({1, 1, Opcode::Arm, AckResult::Rejected});
    for (int i = 0; i < 5; ++i) {
        const auto dup = rx.duplicate_of(cmd);
        ASSERT_TRUE(dup.has_value());
        EXPECT_EQ(dup->result, AckResult::Rejected);
        EXPECT_EQ(dup->cmd_id, 1u);
    }
    EXPECT_EQ(rx.accepted(), 1u);
    EXPECT_EQ(rx.duplicates(), 5u);
}

TEST(CommandReceiver, NewSessionResetsIdSpace) {
    fc::CommandReceiver rx;
    ASSERT_FALSE(rx.duplicate_of({1, 5, Opcode::Ping, 0, 0}));
    rx.record({1, 5, Opcode::Ping, AckResult::Ok});
    // Ground station restarted with a new session: id 1 is new again.
    EXPECT_FALSE(rx.duplicate_of({2, 1, Opcode::Ping, 0, 0}).has_value());
}

TEST(CommandReceiver, OldIdOutsideHistoryIsStillNotReexecuted) {
    fc::CommandReceiver rx;
    for (std::uint32_t id = 1; id <= 40; ++id) {
        ASSERT_FALSE(rx.duplicate_of({1, id, Opcode::Ping, 0, 0}));
        rx.record({1, id, Opcode::Ping, AckResult::Ok});
    }
    const auto dup = rx.duplicate_of({1, 3, Opcode::Ping, 0, 0});
    ASSERT_TRUE(dup.has_value());
    EXPECT_EQ(dup->result, AckResult::Rejected);
}

// Command handling inside the flight computer.
class FcCommandTest : public ::testing::Test {
protected:
    // Feed healthy pad sensors so ARM is allowed.
    void SetUp() override {
        for (int i = 0; i < 30; ++i) step({});
    }

    std::vector<Ack> step(std::vector<Command> cmds) {
        fc::proto::SensorSample s;
        t_us_ += 10'000;
        s.sim_time_us = t_us_;
        s.accel_mps2 = static_cast<float>(fc::kG0) + ((++n_ % 2) ? 0.01f : -0.01f);
        s.flags = fc::proto::kSensorImuValid;
        if (n_ % 5 == 0) {
            s.flags |= fc::proto::kSensorAltValid;
            s.alt_m = 0.0f;
        }
        const auto& out = fc_.step({std::span<const fc::proto::SensorSample>(&s, 1), cmds});
        return {out.ack_span().begin(), out.ack_span().end()};
    }

    Command cmd(Opcode op, ParamId param = ParamId::AscentBurnS, float value = 0.0f) {
        return {77, next_id_++, op, static_cast<std::uint8_t>(param), value};
    }

    fc::FlightComputer fc_;
    std::uint64_t t_us_ = 0;
    int n_ = 0;
    std::uint32_t next_id_ = 1;
};

TEST_F(FcCommandTest, DuplicateArmExecutesOnce) {
    const Command arm = cmd(Opcode::Arm);
    auto acks = step({arm, arm, arm});
    ASSERT_EQ(acks.size(), 3u);
    for (const auto& a : acks) EXPECT_EQ(a.result, AckResult::Ok);
    EXPECT_TRUE(fc_.state_machine().armed());
    EXPECT_EQ(fc_.step({}).telemetry.cmds_accepted, 1u);
}

TEST_F(FcCommandTest, PingIsAlwaysAcked) {
    const auto acks = step({cmd(Opcode::Ping)});
    ASSERT_EQ(acks.size(), 1u);
    EXPECT_EQ(acks[0].result, AckResult::Ok);
}

TEST_F(FcCommandTest, SetParamValidatesRange) {
    EXPECT_EQ(step({cmd(Opcode::SetParam, ParamId::LandingKp, 2.0f)})[0].result, AckResult::Ok);
    EXPECT_DOUBLE_EQ(fc_.config().guidance.kp, 2.0);
    EXPECT_EQ(step({cmd(Opcode::SetParam, ParamId::LandingKp, 50.0f)})[0].result, AckResult::BadParam);
    EXPECT_DOUBLE_EQ(fc_.config().guidance.kp, 2.0);
    EXPECT_EQ(step({cmd(Opcode::SetParam, static_cast<ParamId>(99), 1.0f)})[0].result, AckResult::BadParam);
}

TEST_F(FcCommandTest, AscentBurnLockedOnceArmed) {
    EXPECT_EQ(step({cmd(Opcode::SetParam, ParamId::AscentBurnS, 8.0f)})[0].result, AckResult::Ok);
    EXPECT_DOUBLE_EQ(fc_.state_machine().config().ascent_burn_s, 8.0);
    step({cmd(Opcode::Arm)});
    EXPECT_EQ(step({cmd(Opcode::SetParam, ParamId::AscentBurnS, 4.0f)})[0].result, AckResult::Rejected);
    EXPECT_DOUBLE_EQ(fc_.state_machine().config().ascent_burn_s, 8.0);
}

TEST_F(FcCommandTest, UnknownOpcodeRejected) {
    Command bad = cmd(Opcode::Ping);
    bad.opcode = static_cast<Opcode>(200);
    EXPECT_EQ(step({bad})[0].result, AckResult::BadOpcode);
}

TEST_F(FcCommandTest, AbortCommandAborts) {
    step({cmd(Opcode::Arm)});
    for (int i = 0; i < 310; ++i) step({});
    ASSERT_EQ(fc_.state(), fc::FlightState::Ascent);
    EXPECT_EQ(step({cmd(Opcode::Abort)})[0].result, AckResult::Ok);
    EXPECT_EQ(fc_.state(), fc::FlightState::Abort);
    const auto& out = fc_.step({});
    EXPECT_EQ(out.actuator.engine_on, 0);
    EXPECT_EQ(out.actuator.chute, 1);
}

}  // namespace
