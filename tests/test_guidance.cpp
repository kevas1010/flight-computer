#include <gtest/gtest.h>

#include <cmath>

#include "fc/guidance.hpp"

namespace {

const fc::VehicleParams kVehicle;
const fc::GuidanceConfig kGuidance;

TEST(Guidance, NoIgnitionWhileAscending) {
    EXPECT_TRUE(std::isinf(fc::ignition_altitude(kVehicle, kGuidance, 5.0, 900.0)));
    EXPECT_LT(fc::ignition_altitude(kVehicle, kGuidance, 5.0, 900.0), 0.0);
}

TEST(Guidance, IgnitionAltitudeMatchesKinematics) {
    const double mass = 900.0;
    const double v = -80.0;
    const double a_net = kGuidance.plan_throttle * kVehicle.max_thrust_n / mass - fc::kG0;
    const double v_excess = -v - kGuidance.terminal_velocity;
    const double expected = v_excess * v_excess / (2 * a_net) + kGuidance.terminal_alt_m + kGuidance.ignition_margin_m;
    EXPECT_NEAR(fc::ignition_altitude(kVehicle, kGuidance, v, mass), expected, 1e-9);
}

TEST(Guidance, IgnitionAltitudeGrowsWithSpeedAndMass) {
    EXPECT_GT(fc::ignition_altitude(kVehicle, kGuidance, -100.0, 900.0),
              fc::ignition_altitude(kVehicle, kGuidance, -60.0, 900.0));
    EXPECT_GT(fc::ignition_altitude(kVehicle, kGuidance, -80.0, 1000.0),
              fc::ignition_altitude(kVehicle, kGuidance, -80.0, 700.0));
}

TEST(Guidance, ImpossibleDecelerationIgnitesImmediately) {
    EXPECT_TRUE(std::isinf(fc::ignition_altitude(kVehicle, kGuidance, -10.0, 5000.0)));
    EXPECT_GT(fc::ignition_altitude(kVehicle, kGuidance, -10.0, 5000.0), 0.0);
}

TEST(Guidance, ProfileStartsAtIgnitionState) {
    fc::LandingController lc;
    lc.start(kGuidance, 300.0, -70.0);
    EXPECT_NEAR(lc.v_ref(kGuidance, 300.0), -70.0, 1e-9);
    EXPECT_NEAR(lc.v_ref(kGuidance, kGuidance.terminal_alt_m), -kGuidance.terminal_velocity, 1e-9);
    EXPECT_NEAR(lc.v_ref(kGuidance, 0.0), -kGuidance.terminal_velocity, 1e-9);
}

TEST(Guidance, ThrottleRisesWhenFallingFasterThanProfile) {
    fc::LandingController lc;
    lc.start(kGuidance, 300.0, -70.0);
    const double on_profile = lc.throttle(kVehicle, kGuidance, 200.0, lc.v_ref(kGuidance, 200.0), 900.0);
    const double too_fast = lc.throttle(kVehicle, kGuidance, 200.0, lc.v_ref(kGuidance, 200.0) - 5.0, 900.0);
    EXPECT_GT(too_fast, on_profile);
}

TEST(Guidance, ThrottleClampedToEngineLimits) {
    fc::LandingController lc;
    lc.start(kGuidance, 300.0, -70.0);
    EXPECT_DOUBLE_EQ(lc.throttle(kVehicle, kGuidance, 100.0, -200.0, 900.0), 1.0);
    EXPECT_DOUBLE_EQ(lc.throttle(kVehicle, kGuidance, 100.0, 20.0, 900.0), kVehicle.min_throttle);
}

TEST(Guidance, DragOpposesVelocity) {
    EXPECT_GT(fc::drag_force(kVehicle, 100.0, -50.0, false), 0.0);
    EXPECT_LT(fc::drag_force(kVehicle, 100.0, 50.0, false), 0.0);
    EXPECT_GT(fc::drag_force(kVehicle, 100.0, -50.0, true), fc::drag_force(kVehicle, 100.0, -50.0, false));
    EXPECT_LT(fc::air_density(5000.0), fc::air_density(0.0));
}

}  // namespace
