#include <gtest/gtest.h>

#include <random>

#include "fc/faults.hpp"
#include "fc/flight_computer.hpp"

namespace {

TEST(ImuStuckDetector, NoisySignalNeverFlagged) {
    fc::ImuStuckDetector d(5, 50);
    std::mt19937 rng(7);
    std::normal_distribution<float> n(9.8f, 0.05f);
    for (int i = 0; i < 100000; ++i) ASSERT_FALSE(d.update(n(rng)));
}

TEST(ImuStuckDetector, FlagsAfterIdenticalSamples) {
    fc::ImuStuckDetector d(5, 50);
    d.update(1.0f);
    for (int i = 0; i < 3; ++i) EXPECT_FALSE(d.update(9.75f)) << i;
    d.update(9.75f);
    d.update(9.75f);
    EXPECT_TRUE(d.stuck());
}

TEST(ImuStuckDetector, ClearsOnlyAfterSustainedRecovery) {
    fc::ImuStuckDetector d(5, 50);
    for (int i = 0; i < 10; ++i) d.update(3.0f);
    ASSERT_TRUE(d.stuck());
    for (int i = 0; i < 49; ++i) EXPECT_TRUE(d.update(static_cast<float>(i)));
    EXPECT_FALSE(d.update(1000.0f));
}

// Flight-computer-level fault detection from the sensor stream.
class FcFaultTest : public ::testing::Test {
protected:
    fc::proto::SensorSample sample(bool alt) {
        fc::proto::SensorSample s;
        t_us_ += 10'000;
        s.sim_time_us = t_us_;
        s.accel_mps2 = static_cast<float>(fc::kG0 + noise_(rng_));
        s.flags = fc::proto::kSensorImuValid;
        if (alt) {
            s.alt_m = static_cast<float>(noise_(rng_));
            s.flags |= fc::proto::kSensorAltValid;
        }
        return s;
    }

    void tick(const fc::proto::SensorSample* s) {
        fc_.step({s != nullptr ? std::span<const fc::proto::SensorSample>(s, 1) : std::span<const fc::proto::SensorSample>(),
                  {}});
    }

    fc::FlightComputer fc_;
    std::uint64_t t_us_ = 0;
    std::mt19937 rng_{11};
    std::normal_distribution<double> noise_{0.0, 0.05};
};

TEST_F(FcFaultTest, StartsWithLinkAndAltimeterFaultsUntilDataArrives) {
    tick(nullptr);
    EXPECT_NE(fc_.faults() & fc::kFaultSensorLink, 0);
    for (int i = 0; i < 20; ++i) {
        const auto s = sample(i % 5 == 0);
        tick(&s);
    }
    EXPECT_EQ(fc_.faults(), 0);
}

TEST_F(FcFaultTest, AltimeterDropoutDetectedAfter250ms) {
    for (int i = 0; i < 50; ++i) {
        const auto s = sample(i % 5 == 0);
        tick(&s);
    }
    ASSERT_EQ(fc_.faults(), 0);
    for (int i = 0; i < 26; ++i) {
        const auto s = sample(false);
        tick(&s);
    }
    EXPECT_NE(fc_.faults() & fc::kFaultAltStale, 0);
}

TEST_F(FcFaultTest, SensorLinkLossDetectedAfter200ms) {
    for (int i = 0; i < 50; ++i) {
        const auto s = sample(i % 5 == 0);
        tick(&s);
    }
    for (int i = 0; i < 20; ++i) tick(nullptr);
    EXPECT_EQ(fc_.faults() & fc::kFaultSensorLink, 0);
    tick(nullptr);
    EXPECT_NE(fc_.faults() & fc::kFaultSensorLink, 0);
}

TEST_F(FcFaultTest, StuckImuDetectedFromSensorStream) {
    for (int i = 0; i < 50; ++i) {
        const auto s = sample(i % 5 == 0);
        tick(&s);
    }
    auto frozen = sample(false);
    for (int i = 0; i < 6; ++i) {
        frozen.sim_time_us = (t_us_ += 10'000);
        tick(&frozen);
    }
    EXPECT_NE(fc_.faults() & fc::kFaultImuStuck, 0);
}

TEST_F(FcFaultTest, ReorderedSamplesAreDropped) {
    for (int i = 0; i < 50; ++i) {
        const auto s = sample(i % 5 == 0);
        tick(&s);
    }
    const double h = fc_.filter().h();
    auto old = sample(true);
    old.sim_time_us = 1000;  // older than everything seen
    old.alt_m = 500.0f;
    tick(&old);
    EXPECT_DOUBLE_EQ(fc_.filter().h(), h);
}

}  // namespace
