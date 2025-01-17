#pragma once

#include <Eigen/Core>
#include <iostream>
#include <sophus/se3.hpp>

namespace kiss_icp {
struct State {
    using Vector6d = Sophus::SE3d::Tangent;

    Sophus::SE3d poseAtNormalizedTime(const double tau) const;

    inline const auto &coefficients() const { return coefficients_; }

    void update(const Vector6d &dx);

    inline void computeNextState() {
        pose = pose * Sophus::SE3d::exp(coefficients_[0]);
        coefficients_[0] = coefficients_[1];
        coefficients_[1] = coefficients_[2];
        // coefficients_[2] = Vector6d::Zero();
    }

    inline Eigen::Vector3d transformPoint(const Eigen::Vector3d &point, const double tau) const {
        return poseAtNormalizedTime(tau) * point;
    }

protected:
    Sophus::SE3d pose;
    std::array<Vector6d, 3> coefficients_{Vector6d::Zero(), Vector6d::Zero(), Vector6d::Zero()};
};
}  // namespace kiss_icp
