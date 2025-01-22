#include "State.hpp"

#include <sophus/so3.hpp>

namespace {

inline double square(const double x) { return x * x; }

}  // namespace

namespace kiss_icp {
void State::update(const State::Vector6d &dx) {
    const auto &tau = coefficients_[1];
    const auto Jr_inverse = Sophus::SE3d::leftJacobianInverse(-tau);
    coefficients_[1] += Jr_inverse * dx;
}
void State::computeNextState() {
    pose = poseAtNormalizedTime(1.0);
    const Sophus::SE3d A0_inv = Sophus::SE3d::exp(coefficients_[1]).inverse();
    coefficients_[0] = 2 * coefficients_[1] + A0_inv.Adj() * coefficients_[0];
    coefficients_[1].setZero();
}

Sophus::SE3d State::poseAtNormalizedTime(const double tau) const {
    const Sophus::SE3d B = Sophus::SE3d::exp(coefficients_[0] * tau);
    const Sophus::SE3d A = Sophus::SE3d::exp(coefficients_[1] * square(tau));
    return pose * B * A;
}
}  // namespace kiss_icp
