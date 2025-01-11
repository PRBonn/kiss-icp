#include "State.hpp"

#include <iostream>
#include <sophus/so3.hpp>

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
    return 2.0 * acceleration_coefficient * alpha + velocity_coefficient;
}

void State::computeNextState() {
    pose = poseAtNormalizedTime(1.0);
    velocity_coefficient = velocityAtNormalizedTime(1.0);
}
}  // namespace kiss_icp
