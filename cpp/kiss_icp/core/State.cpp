#include "State.hpp"

#include <iostream>
#include <sophus/so3.hpp>

namespace {

inline double square(const double x) { return x * x; }

}  // namespace

namespace kiss_icp {
void State::update(const State::Vector6d &dx) { coefficients_[1] += dx; }
void State::computeNextState() {
    pose = poseAtNormalizedTime(1.0);
    const auto &[b0, a0] = coefficients();
    const Sophus::SE3d &A0 = Sophus::SE3d::exp(a0);
    const Sophus::SE3d &A0_inv = A0.inverse();
    const auto &Adj = A0_inv.Adj();
    coefficients_[0] = 2 * a0 + Adj * b0;
    coefficients_[1].setZero();
}

Sophus::SE3d State::poseAtNormalizedTime(const double tau) const {
    const Sophus::SE3d B = Sophus::SE3d::exp(coefficients_[0] * tau);
    const Sophus::SE3d A = Sophus::SE3d::exp(coefficients_[1] * square(tau));
    return pose * B * A;
}
}  // namespace kiss_icp
