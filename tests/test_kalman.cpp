#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "fc/kalman.hpp"

namespace {

constexpr double kDt = 0.01;

TEST(Kalman, ConvergesOnStaticAltitude) {
    fc::AltitudeKalman kf;
    std::mt19937 rng(1);
    std::normal_distribution<double> alt_noise(0.0, 0.25);
    std::normal_distribution<double> acc_noise(0.0, 0.05);
    for (int i = 0; i < 2000; ++i) {
        kf.predict(acc_noise(rng), kDt, 0.08, true);
        // The first reading initialises the filter, so it bypasses the innovation gate.
        if (i % 5 == 0) kf.update_altitude(50.0 + alt_noise(rng), i > 0);
    }
    EXPECT_NEAR(kf.h(), 50.0, 0.2);
    EXPECT_NEAR(kf.v(), 0.0, 0.1);
    EXPECT_LT(kf.h_sigma(), 0.2);
}

TEST(Kalman, EstimatesAccelerometerBiasOnThePad) {
    fc::AltitudeKalman kf;
    std::mt19937 rng(2);
    std::normal_distribution<double> alt_noise(0.0, 0.25);
    std::normal_distribution<double> acc_noise(0.0, 0.05);
    constexpr double kBias = 0.12;
    for (int i = 0; i < 6000; ++i) {
        kf.predict(kBias + acc_noise(rng), kDt, 0.08, true);
        if (i % 5 == 0) kf.update_altitude(alt_noise(rng));
    }
    EXPECT_NEAR(kf.bias(), kBias, 0.03);
    EXPECT_NEAR(kf.v(), 0.0, 0.05);
}

TEST(Kalman, TracksConstantAccelerationClimb) {
    fc::AltitudeKalman kf;
    std::mt19937 rng(3);
    std::normal_distribution<double> alt_noise(0.0, 0.25);
    std::normal_distribution<double> acc_noise(0.0, 0.05);
    constexpr double kA = 12.0;
    double max_err = 0.0;
    for (int i = 1; i <= 800; ++i) {
        const double t = i * kDt;
        kf.predict(kA + acc_noise(rng), kDt, 0.08, true);
        if (i % 5 == 0) kf.update_altitude(0.5 * kA * t * t + alt_noise(rng));
        max_err = std::max(max_err, std::abs(kf.h() - 0.5 * kA * t * t));
    }
    EXPECT_LT(max_err, 0.5);
    EXPECT_NEAR(kf.v(), kA * 8.0, 0.3);
}

TEST(Kalman, InnovationGateRejectsOutlierAndLeavesStateAlone) {
    fc::AltitudeKalman kf;
    for (int i = 0; i < 500; ++i) {
        kf.predict(0.0, kDt, 0.08, true);
        if (i % 5 == 0) kf.update_altitude(10.0);
    }
    const double h_before = kf.h();
    EXPECT_FALSE(kf.update_altitude(60.0));
    EXPECT_GT(kf.last_nis(), 25.0);
    EXPECT_DOUBLE_EQ(kf.h(), h_before);
    // Ungated, the same reading is applied.
    EXPECT_TRUE(kf.update_altitude(60.0, false));
    EXPECT_GT(kf.h(), h_before);
}

TEST(Kalman, UncertaintyGrowsDuringAltimeterDropout) {
    fc::AltitudeKalman kf;
    for (int i = 0; i < 500; ++i) {
        kf.predict(0.0, kDt, 0.08, true);
        if (i % 5 == 0) kf.update_altitude(0.0);
    }
    const double sigma_before = kf.h_sigma();
    for (int i = 0; i < 500; ++i) kf.predict(0.0, kDt, 0.08, true);
    EXPECT_GT(kf.h_sigma(), 2.0 * sigma_before);
}

TEST(Kalman, CovarianceStaysSymmetricPositiveDefinite) {
    fc::AltitudeKalman kf;
    std::mt19937 rng(4);
    std::normal_distribution<double> n(0.0, 1.0);
    for (int i = 0; i < 20000; ++i) {
        kf.predict(n(rng), kDt, (i / 1000) % 2 == 0 ? 0.08 : 1.5, (i / 3000) % 2 == 0);
        if (i % 5 == 0) kf.update_altitude(n(rng), false);
        if (i % 50 == 0) kf.update_velocity(0.0, 0.2);
    }
    const auto& p = kf.covariance();
    for (int i = 0; i < 3; ++i) {
        EXPECT_GT(p[i][i], 0.0);
        for (int j = 0; j < 3; ++j) EXPECT_NEAR(p[i][j], p[j][i], 1e-9);
    }
    // Leading principal minors positive (Sylvester's criterion).
    EXPECT_GT(p[0][0] * p[1][1] - p[0][1] * p[1][0], 0.0);
}

TEST(Kalman, ModelDrivenPredictionDoesNotMoveBias) {
    fc::AltitudeKalman kf;
    for (int i = 0; i < 2000; ++i) {
        kf.predict(0.1, kDt, 0.08, true);
        if (i % 5 == 0) kf.update_altitude(0.0);
    }
    const double bias = kf.bias();
    for (int i = 0; i < 100; ++i) kf.predict(0.0, kDt, 1.5, false);
    EXPECT_NEAR(kf.bias(), bias, 1e-3);
    EXPECT_NEAR(kf.v(), 0.0, 0.05);  // u was the true accel; bias not subtracted
}

TEST(Kalman, ZeroVelocityUpdatePullsVelocityToZero) {
    fc::AltitudeKalman kf;
    kf.reset(0.0, -1.5);
    for (int i = 0; i < 50; ++i) {  // 0.5 s of ground contact at 100 Hz
        kf.predict(0.0, kDt, 1.5, false);
        kf.update_velocity(0.0, 0.2);
    }
    EXPECT_NEAR(kf.v(), 0.0, 0.1);
}

}  // namespace
