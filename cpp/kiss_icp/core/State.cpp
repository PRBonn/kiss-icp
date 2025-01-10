#include "State.hpp"

namespace {
double square(const double x) { return x * x; }
}  // namespace

namespace kiss_icp {
Sophus::SE3d State::poseAtNormalizedTime(const double alpha) {
    const State::Vector6d &omega =
        acceleration_coefficient * square(alpha) + velocity_coefficient * alpha;
    return pose * Sophus::SE3d::exp(omega);
}

State::Vector6d State::velocityAtNormalizedTime(const double alpha) {
    const auto Jr_inverse = Sophus::SE3d::leftJacobianInverse(-pose.log());
    return Jr_inverse * (2.0 * acceleration_coefficient * alpha + velocity_coefficient);
}

State::JacobianMatrixType State::jacobianTransformPoint(const Eigen::Vector3d &point,
                                                        const double alpha) {
    State::JacobianMatrixType J_icp = State::JacobianMatrixType::Zero();
    J_icp.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    J_icp.block<3, 3>(0, 3) = -Sophus::SO3d::hat(point);
    const Eigen::Matrix3d &rotation_at_alpha = poseAtNormalizedTime(alpha).so3().rotationMatrix();
    const State::Vector6d &omega_at_alpha =
        acceleration_coefficient * square(alpha) + velocity_coefficient * alpha;
    const auto Jr = Sophus::SE3d::leftJacobian(-omega_at_alpha);
    return rotation_at_alpha * J_icp * Jr * std::pow(alpha, 3);
}

void State::computeNextState() {
    const auto &new_pose = poseAtNormalizedTime(1.0);
    velocity_coefficient = velocityAtNormalizedTime(1.0);
    pose = new_pose;
}
}  // namespace kiss_icp
