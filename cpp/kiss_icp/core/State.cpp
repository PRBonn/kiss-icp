#include "State.hpp"

#include <sophus/so3.hpp>

namespace {

inline double square(const double x) { return x * x; }
inline double cube(const double x) { return square(x) * x; }

// From https://bmva-archive.org.uk/bmvc/2013/Papers/paper0093/paper0093.pdf
static const Eigen::Matrix<double, 3, 4> C{{5, 3, -3, 1}, {1, 3, 3, -2}, {0, 0, 0, 1}};

inline Eigen::Vector3d BSplines(const double tau) {
    const Eigen::Matrix<double, 4, 1> u(1, tau, square(tau), cube(tau));
    return 1.0 / 6.0 * C * u;
}
}  // namespace

namespace kiss_icp {
void State::update(const State::Vector6d &dx) {
    const auto &tau = coefficients_[2];
    const auto Jr_inverse = Sophus::SE3d::leftJacobianInverse(-tau);
    coefficients_[2] += Jr_inverse * dx;
}
Sophus::SE3d State::poseAtNormalizedTime(const double tau) const {
    const auto splines = BSplines(tau);
    const Sophus::SE3d a0 = Sophus::SE3d::exp(coefficients_[0] * splines[0]);
    const Sophus::SE3d a1 = Sophus::SE3d::exp(coefficients_[1] * splines[1]);
    const Sophus::SE3d a2 = Sophus::SE3d::exp(coefficients_[2] * splines[2]);
    return pose * a0 * a1 * a2;
}
}  // namespace kiss_icp
