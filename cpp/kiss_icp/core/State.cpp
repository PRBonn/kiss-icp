#include "State.hpp"

#include <iostream>
#include <sophus/so3.hpp>

namespace {
double square(const double x) { return x * x; }
// double cube(const double x) { return x * x * x; }

Sophus::SE3d v2t(const Sophus::SE3d::Tangent &v) {
    Sophus::SE3d T;
    T.so3() = Sophus::SO3d::exp(v.tail<3>());
    T.translation() = v.head<3>();
    return T;
}
}  // namespace

namespace kiss_icp {
State::Vector6d State::relativeMotionVectorAtNormalizedTime(const double tau) const {
    // const auto &[a, b, c] = coefficients();
    // return a * cube(tau) + b * square(tau) + c * tau;
    return coefficients_[0] * tau;
}

Sophus::SE3d State::poseAtNormalizedTime(const double tau) const {
    State::Vector6d omega = relativeMotionVectorAtNormalizedTime(tau);
    return pose * v2t(omega);
}

State::Vector6d State::velocityAtNormalizedTime(const double tau) const {
    const auto &[a, b, c] = coefficients();
    return 3.0 * a * square(tau) + 2.0 * b * tau + c;
}

State::Vector6d State::accelerationAtNormalizedTime(const double tau) const {
    const auto &[a, b, c] = coefficients();
    return 0.5 * (6.0 * a * tau + 2.0 * b);
}
void State::updateCoefficients(const Vector6d &dx) {
    const auto &omega = relativeMotionVectorAtNormalizedTime(1.0);
    const Eigen::Matrix3d Jr_inverse =
        Sophus::SO3d::leftJacobianInverse(omega.template tail<3>()).transpose();
    coefficients_[0].head<3>() += dx.head<3>();
    coefficients_[0].tail<3>() += Jr_inverse * dx.tail<3>();
}

void State::computeNextState() {
    pose = poseAtNormalizedTime(1.0);
    // const State::Vector6d b0 = accelerationAtNormalizedTime(1.0);
    // const State::Vector6d c0 = velocityAtNormalizedTime(1.0);
    // coefficients_[1] = 0.01 * b0;
    // coefficients_[2] = 0.1 * c0;
}
}  // namespace kiss_icp
