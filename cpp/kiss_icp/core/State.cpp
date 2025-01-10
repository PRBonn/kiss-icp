#include "State.hpp"

namespace {
double square(const double x) { return x * x; }
}  // namespace

namespace kiss_icp {
State::Vector6d State::relativeMotionVectorAtNormalizedTime(const double alpha) const {
    return acceleration_coefficient * square(alpha) + velocity_coefficient * alpha;
}

Sophus::SE3d State::poseAtNormalizedTime(const double alpha) const {
    const State::Vector6d &omega = relativeMotionVectorAtNormalizedTime(alpha);
    return pose * Sophus::SE3d::exp(omega);
}

State::Vector6d State::velocityAtNormalizedTime(const double alpha) const {
    const auto Jr_inverse = Sophus::SE3d::leftJacobianInverse(-pose.log());
    return Jr_inverse * (2.0 * acceleration_coefficient * alpha + velocity_coefficient);
}

void State::computeNextState() {
    const auto &new_pose = poseAtNormalizedTime(1.0);
    velocity_coefficient = velocityAtNormalizedTime(1.0);
    pose = new_pose;
}
}  // namespace kiss_icp
