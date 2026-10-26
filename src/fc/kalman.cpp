#include "fc/kalman.hpp"

#include <cmath>

namespace fc {
namespace {

constexpr double sq(double x) { return x * x; }

Mat3 identity() {
    Mat3 m{};
    for (int i = 0; i < 3; ++i) m[i][i] = 1.0;
    return m;
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
    Mat3 c{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) c[i][j] += a[i][k] * b[k][j];
    return c;
}

Mat3 transpose(const Mat3& a) {
    Mat3 t{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) t[i][j] = a[j][i];
    return t;
}

// A * P * A^T
Mat3 sandwich(const Mat3& a, const Mat3& p) { return multiply(multiply(a, p), transpose(a)); }

}  // namespace

AltitudeKalman::AltitudeKalman(const KalmanConfig& cfg) : cfg_(cfg) { reset(0.0, 0.0); }

void AltitudeKalman::reset(double h, double v) {
    x_ = {h, v, 0.0};
    P_ = {};
    P_[0][0] = sq(cfg_.init_h_sigma);
    P_[1][1] = sq(cfg_.init_v_sigma);
    P_[2][2] = sq(cfg_.init_b_sigma);
    nis_ = 0.0;
}

void AltitudeKalman::predict(double u, double dt, double accel_sigma, bool apply_bias) {
    const double a = u - (apply_bias ? x_[2] : 0.0);
    x_[0] += x_[1] * dt + 0.5 * a * dt * dt;
    x_[1] += a * dt;

    Mat3 f = identity();
    f[0][1] = dt;
    if (apply_bias) {
        f[0][2] = -0.5 * dt * dt;
        f[1][2] = -dt;
    }
    P_ = sandwich(f, P_);

    // Acceleration noise enters through G = [dt^2/2, dt, 0]^T.
    const double g0 = 0.5 * dt * dt;
    const double g1 = dt;
    const double q = sq(accel_sigma);
    P_[0][0] += q * g0 * g0;
    P_[0][1] += q * g0 * g1;
    P_[1][0] += q * g0 * g1;
    P_[1][1] += q * g1 * g1;
    P_[2][2] += sq(cfg_.bias_walk_sigma) * dt;
}

bool AltitudeKalman::update_altitude(double z, bool gate) {
    const double r = sq(cfg_.alt_sigma);
    const double y = z - x_[0];
    const double s = P_[0][0] + r;
    nis_ = y * y / s;
    if (gate && nis_ > sq(cfg_.gate_sigma)) return false;

    std::array<double, 3> k{};
    for (int i = 0; i < 3; ++i) k[i] = P_[i][0] / s;
    for (int i = 0; i < 3; ++i) x_[i] += k[i] * y;

    // Joseph form keeps P symmetric positive-definite: (I-KH) P (I-KH)^T + K R K^T.
    Mat3 a = identity();
    for (int i = 0; i < 3; ++i) a[i][0] -= k[i];
    P_ = sandwich(a, P_);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P_[i][j] += r * k[i] * k[j];
    return true;
}

void AltitudeKalman::update_velocity(double z, double sigma) {
    const double r = sq(sigma);
    const double y = z - x_[1];
    const double s = P_[1][1] + r;

    std::array<double, 3> k{};
    for (int i = 0; i < 3; ++i) k[i] = P_[i][1] / s;
    for (int i = 0; i < 3; ++i) x_[i] += k[i] * y;

    Mat3 a = identity();
    for (int i = 0; i < 3; ++i) a[i][1] -= k[i];
    P_ = sandwich(a, P_);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P_[i][j] += r * k[i] * k[j];
}

double AltitudeKalman::h_sigma() const { return std::sqrt(P_[0][0]); }
double AltitudeKalman::v_sigma() const { return std::sqrt(P_[1][1]); }

}  // namespace fc
