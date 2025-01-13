#include "State.hpp"

#include <iostream>
#include <sophus/so3.hpp>

namespace kiss_icp {

Sophus::SE3d State::poseAtNormalizedTime(const double tau) const {
    State::Vector6d omega = tau * coefficient_;
    return pose * Sophus::SE3d::exp(omega);
}
void State::updateCoefficients(const Vector6d &dx) { coefficient_ += dx; }

void State::computeNextState() { pose = poseAtNormalizedTime(1.0); }
}  // namespace kiss_icp
