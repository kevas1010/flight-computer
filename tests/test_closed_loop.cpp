// Full flights through the in-process closed loop (vehicle physics + flight computer).

#include <gtest/gtest.h>

#include "sim/closed_loop.hpp"

namespace {

using fc::FlightState;

sim::ScenarioResult fly(const std::string& name) {
    const auto sc = sim::named_scenario(name);
    EXPECT_TRUE(sc.has_value()) << name;
    return sim::run_closed_loop(*sc);
}

bool visited(const sim::ScenarioResult& r, FlightState s) {
    for (const auto& t : r.transitions)
        if (t.state == s) return true;
    return false;
}

TEST(ClosedLoop, NominalFlightLandsSoftly) {
    const auto r = fly("nominal");
    EXPECT_EQ(r.final_state, FlightState::Landed);
    EXPECT_TRUE(r.touched_down);
    EXPECT_LT(r.touchdown_speed, 2.0);
    EXPECT_GT(r.max_altitude, 300.0);
    EXPECT_FALSE(r.chute);
    EXPECT_EQ(r.faults_seen, 0);
    EXPECT_LT(r.max_nav_error_m, 1.0);
    for (const auto s : {FlightState::Ascent, FlightState::Coast, FlightState::Descent, FlightState::LandingBurn})
        EXPECT_TRUE(visited(r, s)) << fc::to_string(s);
}

TEST(ClosedLoop, NominalFlightIsDeterministic) {
    const auto a = fly("nominal");
    const auto b = fly("nominal");
    EXPECT_EQ(a.touchdown_speed, b.touchdown_speed);
    EXPECT_EQ(a.end_time_s, b.end_time_s);
    EXPECT_EQ(a.transitions.size(), b.transitions.size());
}

TEST(ClosedLoop, LandsAcrossSensorNoiseSeeds) {
    for (std::uint64_t seed = 1; seed <= 10; ++seed) {
        auto sc = *sim::named_scenario("nominal");
        sc.seed = seed;
        const auto r = sim::run_closed_loop(sc);
        EXPECT_EQ(r.final_state, FlightState::Landed) << "seed " << seed;
        EXPECT_LT(r.touchdown_speed, 2.0) << "seed " << seed;
    }
}

TEST(ClosedLoop, AltimeterDropoutDegradesToImuAndStillLands) {
    const auto r = fly("alt_dropout");
    EXPECT_NE(r.faults_seen & fc::kFaultAltStale, 0);
    EXPECT_GT(r.alt_stale_frames, 500);  // ~6 s dead-reckoning on the IMU
    EXPECT_EQ(r.final_state, FlightState::Landed);
    EXPECT_LT(r.touchdown_speed, 2.0);
    EXPECT_LT(r.max_nav_error_m, 2.0);
}

TEST(ClosedLoop, StuckImuDegradesToThrustModelAndStillLands) {
    const auto r = fly("imu_stuck");
    EXPECT_NE(r.faults_seen & fc::kFaultImuStuck, 0);
    EXPECT_GT(r.imu_fault_frames, 1000);
    EXPECT_EQ(r.final_state, FlightState::Landed);
    EXPECT_LT(r.touchdown_speed, 2.0);
    EXPECT_LT(r.max_nav_error_m, 3.0);
}

TEST(ClosedLoop, LossOfBothNavSensorsAbortsWithChute) {
    const auto r = fly("dual_fault");
    EXPECT_EQ(r.final_state, FlightState::Abort);
    EXPECT_EQ(r.abort_reason, fc::AbortReason::NavLoss);
    EXPECT_TRUE(r.chute);
    EXPECT_FALSE(visited(r, FlightState::LandingBurn));
}

TEST(ClosedLoop, SensorBlackoutAborts) {
    const auto r = fly("sensor_blackout");
    EXPECT_EQ(r.final_state, FlightState::Abort);
    EXPECT_EQ(r.abort_reason, fc::AbortReason::SensorLink);
    EXPECT_TRUE(r.chute);
}

TEST(ClosedLoop, CommandedAbortDuringAscentCutsEngine) {
    const auto r = fly("abort_ascent");
    EXPECT_EQ(r.final_state, FlightState::Abort);
    EXPECT_EQ(r.abort_reason, fc::AbortReason::Commanded);
    ASSERT_GE(r.transitions.size(), 2u);
    EXPECT_NEAR(r.transitions.back().time_s, 6.0, 0.02);
    EXPECT_TRUE(r.chute);
}

}  // namespace
