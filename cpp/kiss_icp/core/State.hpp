#pragma once

#include <sophus/se3.hpp>

namespace kiss_icp {
struct State {
    using Vector6d = Sophus::SE3d::Tangent;
    Vector6d relativeMotionVectorAtNormalizedTime(const double tau) const;

    Sophus::SE3d poseAtNormalizedTime(const double tau) const;
    Vector6d velocityAtNormalizedTime(const double tau) const;
    Vector6d accelerationAtNormalizedTime(const double tau) const;

    inline void updateCoefficients(const Vector6d &dx) { coefficients.front() += dx; }

    void computeNextState();

    inline Eigen::Vector3d transformPoint(const Eigen::Vector3d &point, const double tau) const {
        return poseAtNormalizedTime(tau) * point;
    }

protected:
    Sophus::SE3d pose;
    std::array<Vector6d, 3> coefficients{Vector6d::Zero(),  //
                                         Vector6d::Zero(),  //
                                         Vector6d::Zero()};
};
}  // namespace kiss_icp
