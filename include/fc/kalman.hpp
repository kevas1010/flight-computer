#pragma once

#include <array>

namespace fc {

struct KalmanConfig {
    double accel_sigma = 0.08;       // IMU white noise, m/s^2 (per sample)
    double bias_walk_sigma = 0.002;  // accelerometer bias random walk, m/s^2/sqrt(s)
    double alt_sigma = 0.25;         // altimeter noise, m
    double gate_sigma = 5.0;         // innovation gate, in standard deviations
    double init_h_sigma = 1.0;
    double init_v_sigma = 0.1;
    double init_b_sigma = 0.3;
};

using Mat3 = std::array<std::array<double, 3>, 3>;

// Three-state vertical navigation filter: x = [altitude, velocity, accel bias].
// The IMU drives the prediction; the altimeter corrects it.
class AltitudeKalman {
public:
    explicit AltitudeKalman(const KalmanConfig& cfg = {});

    void reset(double h, double v);

    // Propagates by `dt` using measured net acceleration `u` = specific force - g.
    // With apply_bias the bias state is subtracted from `u` and kept observable;
    // without it (`u` comes from a thrust model, not the IMU) the bias is frozen.
    void predict(double u, double dt, double accel_sigma, bool apply_bias);

    // Altimeter update. Returns false if `gate` is set and the innovation fails the
    // chi-square gate, in which case the state is left unchanged.
    bool update_altitude(double z, bool gate = true);

    // Velocity pseudo-measurement, e.g. a zero-velocity update when the vehicle is
    // known to be resting on the ground.
    void update_velocity(double z, double sigma);

    double h() const { return x_[0]; }
    double v() const { return x_[1]; }
    double bias() const { return x_[2]; }
    double h_sigma() const;
    double v_sigma() const;
    const Mat3& covariance() const { return P_; }
    double last_nis() const { return nis_; }  // normalised innovation squared of the last update
    const KalmanConfig& config() const { return cfg_; }

private:
    KalmanConfig cfg_;
    std::array<double, 3> x_{};
    Mat3 P_{};
    double nis_ = 0.0;
};

}  // namespace fc
