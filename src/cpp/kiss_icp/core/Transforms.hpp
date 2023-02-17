// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>

namespace Eigen {
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace kiss_icp {

inline Eigen::Isometry3d MakeTransform(const Eigen::Vector3d &xyz, const Eigen::Vector3d &theta) {
    const double angle = theta.norm();
    const Eigen::Vector3d axis = theta.normalized();
    Eigen::AngleAxisd omega(angle, axis);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = omega.toRotationMatrix();
    transform.translation() = xyz;
    return transform;
}

inline Eigen::Isometry3d Vector6dToIsometry3d(const Eigen::Vector6d &x) {
    const Eigen::Vector3d &xyz = x.tail<3>();
    const Eigen::Vector3d &theta = x.head<3>();
    return MakeTransform(xyz, theta);
}

inline void TransformPoints(const Eigen::Isometry3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

inline Eigen::Isometry3d ConcatenateIsometries(const Eigen::Isometry3d &lhs,
                                               const Eigen::Isometry3d &rhs) {
    Eigen::Isometry3d returned = Eigen::Isometry3d::Identity();
    returned.linear() = lhs.rotation() * rhs.rotation();
    returned.translation() = lhs.rotation() * rhs.translation() + lhs.translation();
    return returned;
}
}  // namespace kiss_icp
