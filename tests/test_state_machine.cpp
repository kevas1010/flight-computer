#include <gtest/gtest.h>

#include "fc/faults.hpp"
#include "fc/state_machine.hpp"

namespace {

using fc::FlightState;
using fc::StateMachine;
using fc::StateMachineInputs;

// Helper that drives the state machine into a given state with healthy inputs.
class StateMachineTest : public ::testing::Test {
protected:
    StateMachineInputs healthy() const {
        StateMachineInputs in;
        in.prop_kg = 400.0;
        return in;
    }

    void run(int ticks, const StateMachineInputs& in) {
        for (int i = 0; i < ticks; ++i) sm.update(in);
    }

    void arm_and_launch() {
        auto in = healthy();
        in.arm = true;
        sm.update(in);
        ASSERT_TRUE(sm.armed());
        run(300, healthy());
        ASSERT_EQ(sm.state(), FlightState::Ascent);
    }

    void reach(FlightState target) {
        arm_and_launch();
        if (target == FlightState::Ascent) return;
        auto in = healthy();
        in.v = 50.0;
        run(600, in);
        ASSERT_EQ(sm.state(), FlightState::Coast);
        if (target == FlightState::Coast) return;
        in.v = -1.0;
        in.h = 600.0;
        run(10, in);
        ASSERT_EQ(sm.state(), FlightState::Descent);
        if (target == FlightState::Descent) return;
        in.ignition_alt = 700.0;
        sm.update(in);
        ASSERT_EQ(sm.state(), FlightState::LandingBurn);
    }

    StateMachine sm;
};

TEST_F(StateMachineTest, StartsOnPadDisarmed) {
    EXPECT_EQ(sm.state(), FlightState::Pad);
    EXPECT_FALSE(sm.armed());
    run(1000, healthy());
    EXPECT_EQ(sm.state(), FlightState::Pad);
}

TEST_F(StateMachineTest, ArmStartsThreeSecondCountdownThenIgnites) {
    auto in = healthy();
    in.arm = true;
    sm.update(in);
    EXPECT_TRUE(sm.armed());
    EXPECT_NEAR(sm.countdown_remaining_s(), 3.0, 1e-9);
    run(299, healthy());
    EXPECT_EQ(sm.state(), FlightState::Pad);
    sm.update(healthy());
    EXPECT_EQ(sm.state(), FlightState::Ascent);
}

TEST_F(StateMachineTest, ArmRejectedWithSensorFault) {
    auto in = healthy();
    in.arm = true;
    in.faults = fc::kFaultAltStale;
    EXPECT_FALSE(sm.can_arm(in.faults));
    sm.update(in);
    EXPECT_FALSE(sm.armed());
}

TEST_F(StateMachineTest, TransientAltOutlierDoesNotInhibitArm) {
    EXPECT_TRUE(sm.can_arm(fc::kFaultAltOutlier));
}

TEST_F(StateMachineTest, FaultDuringCountdownHolds) {
    auto in = healthy();
    in.arm = true;
    sm.update(in);
    run(100, healthy());
    auto faulted = healthy();
    faulted.faults = fc::kFaultImuStuck;
    sm.update(faulted);
    EXPECT_FALSE(sm.armed());
    EXPECT_EQ(sm.holds(), 1u);
    run(500, healthy());
    EXPECT_EQ(sm.state(), FlightState::Pad);  // needs a fresh ARM
}

TEST_F(StateMachineTest, MecoAfterBurnTime) {
    arm_and_launch();
    run(599, healthy());
    EXPECT_EQ(sm.state(), FlightState::Ascent);
    sm.update(healthy());
    EXPECT_EQ(sm.state(), FlightState::Coast);
}

TEST_F(StateMachineTest, MecoEarlyOnPropellantReserve) {
    arm_and_launch();
    auto in = healthy();
    in.prop_kg = sm.config().landing_reserve_kg;
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::Coast);
}

TEST_F(StateMachineTest, ApogeeNeedsConsecutiveNegativeVelocity) {
    reach(FlightState::Coast);
    auto down = healthy();
    down.v = -0.1;
    auto up = healthy();
    up.v = 0.1;
    for (int i = 0; i < 50; ++i) sm.update(i % 3 == 0 ? up : down);  // noisy around apogee
    EXPECT_EQ(sm.state(), FlightState::Coast);
    run(10, down);
    EXPECT_EQ(sm.state(), FlightState::Descent);
}

TEST_F(StateMachineTest, LandingBurnStartsAtIgnitionAltitude) {
    reach(FlightState::Descent);
    auto in = healthy();
    in.v = -80.0;
    in.h = 400.0;
    in.ignition_alt = 380.0;
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::Descent);
    in.h = 379.0;
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::LandingBurn);
}

TEST_F(StateMachineTest, LandedAfterConfirmedTouchdown) {
    reach(FlightState::LandingBurn);
    auto in = healthy();
    in.h = 0.2;
    in.v = -0.1;
    run(9, in);
    EXPECT_EQ(sm.state(), FlightState::LandingBurn);
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::Landed);
    run(100, healthy());
    EXPECT_EQ(sm.state(), FlightState::Landed);
}

TEST_F(StateMachineTest, SingleSensorFaultInFlightDegradesWithoutAbort) {
    reach(FlightState::Descent);
    auto in = healthy();
    in.v = -20.0;
    in.h = 500.0;
    in.faults = fc::kFaultAltStale;
    run(200, in);
    EXPECT_EQ(sm.state(), FlightState::Descent);
    in.faults = fc::kFaultImuStuck;
    run(200, in);
    EXPECT_EQ(sm.state(), FlightState::Descent);
}

TEST_F(StateMachineTest, DualNavigationFaultAborts) {
    reach(FlightState::Coast);
    auto in = healthy();
    in.faults = fc::kFaultAltStale | fc::kFaultImuStuck;
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::Abort);
    EXPECT_EQ(sm.abort_reason(), fc::AbortReason::NavLoss);
}

TEST_F(StateMachineTest, SensorLinkLossAborts) {
    reach(FlightState::Ascent);
    auto in = healthy();
    in.faults = fc::kFaultSensorLink;
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::Abort);
    EXPECT_EQ(sm.abort_reason(), fc::AbortReason::SensorLink);
}

TEST_F(StateMachineTest, EstimatorDivergenceAborts) {
    reach(FlightState::Descent);
    auto in = healthy();
    in.faults = fc::kFaultEstDiverged;
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::Abort);
    EXPECT_EQ(sm.abort_reason(), fc::AbortReason::EstDiverged);
}

TEST_F(StateMachineTest, AbortIsTerminal) {
    reach(FlightState::Ascent);
    auto in = healthy();
    in.abort = true;
    sm.update(in);
    ASSERT_EQ(sm.state(), FlightState::Abort);
    auto arm = healthy();
    arm.arm = true;
    run(1000, arm);
    EXPECT_EQ(sm.state(), FlightState::Abort);
}

class AbortFromEveryState : public StateMachineTest, public ::testing::WithParamInterface<FlightState> {};

TEST_P(AbortFromEveryState, CommandedAbortIsHonoured) {
    const FlightState from = GetParam();
    if (from != FlightState::Pad) reach(from);
    auto in = healthy();
    in.abort = true;
    sm.update(in);
    EXPECT_EQ(sm.state(), FlightState::Abort);
    EXPECT_EQ(sm.abort_reason(), fc::AbortReason::Commanded);
}

INSTANTIATE_TEST_SUITE_P(States, AbortFromEveryState,
                         ::testing::Values(FlightState::Pad, FlightState::Ascent, FlightState::Coast,
                                           FlightState::Descent, FlightState::LandingBurn),
                         [](const auto& p) { return std::string(fc::to_string(p.param)); });

}  // namespace
