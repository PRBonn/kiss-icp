#include "State.hpp"

#include <iostream>
#include <sophus/so3.hpp>

namespace {
double square(const double x) { return x * x; }
double cube(const double x) { return x * x * x; }
}  // namespace

namespace kiss_icp {
State::Vector6d State::relativeMotionVectorAtNormalizedTime(const double tau) const {
    const auto &[a, b, c] = coefficients;
    return a * cube(tau) + b * square(tau) + c * tau;
}

Sophus::SE3d State::poseAtNormalizedTime(const double tau) const {
    State::Vector6d omega = relativeMotionVectorAtNormalizedTime(tau);
    const auto &Jl_inv = Sophus::SO3d::leftJacobianInverse(omega.tail<3>());
    omega.head<3>() = Jl_inv * omega.head<3>();
    return pose * Sophus::SE3d::exp(omega);
}

State::Vector6d State::velocityAtNormalizedTime(const double tau) const {
    const auto &[a, b, c] = coefficients;
    return 3.0 * a * square(tau) + 2.0 * b * tau + c;
}

State::Vector6d State::accelerationAtNormalizedTime(const double tau) const {
    const auto &[a, b, c] = coefficients;
    return 6.0 * a * tau + 2.0 * b;
}

void State::computeNextState() {
    pose = poseAtNormalizedTime(1.0);
    coefficients[0] = State::Vector6d::Zero();
    coefficients[1] = 0.5 * accelerationAtNormalizedTime(1.0);
    coefficients[2] = velocityAtNormalizedTime(1.0);
}
}  // namespace kiss_icp
