#pragma once

#include <sophus/se3.hpp>

namespace kiss_icp {
struct State {
    using Vector6d = Sophus::SE3d::Tangent;
    using JacobianMatrixType = Eigen::Matrix<double, 3, 6>;
    Sophus::SE3d poseAtNormalizedTime(const double alpha) const;
    Vector6d relativeMotionVectorAtNormalizedTime(const double) const;
    Vector6d velocityAtNormalizedTime(const double alpha) const;

    inline void updateCoefficients(const Vector6d &dx) { acceleration_coefficient += dx; }

    void computeNextState();

    inline Eigen::Vector3d transformPoint(const Eigen::Vector3d &point, const double alpha) const {
        return poseAtNormalizedTime(alpha) * point;
    }

protected:
    Sophus::SE3d pose;
    Vector6d velocity_coefficient = Vector6d::Zero();
    Vector6d acceleration_coefficient = Vector6d::Zero();
};
}  // namespace kiss_icp
