#pragma once

#include <sophus/se3.hpp>

namespace kiss_icp {
struct State {
    using Vector6d = Sophus::SE3d::Tangent;

    Sophus::SE3d poseAtNormalizedTime(const double tau) const;

    const Vector6d &coefficient() const { return coefficient_; }

    void updateCoefficients(const Vector6d &dx);

    void computeNextState();

    inline Eigen::Vector3d transformPoint(const Eigen::Vector3d &point, const double tau) const {
        return poseAtNormalizedTime(tau) * point;
    }

protected:
    Sophus::SE3d pose;
    Vector6d coefficient_ = Vector6d::Zero();
};
}  // namespace kiss_icp
