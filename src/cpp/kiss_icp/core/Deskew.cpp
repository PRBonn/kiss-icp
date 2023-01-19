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
#include "Deskew.hpp"

#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>
#include <vector>

namespace {
Eigen::Isometry3d MakeTransform(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
    Eigen::AngleAxisd omega(rpy.norm(), rpy.normalized());
    Eigen::Isometry3d transform(omega);
    transform.translation() = xyz;
    return transform;
}
}  // namespace

namespace kiss_icp {

std::tuple<Eigen::Vector3d, Eigen::Vector3d> VelocityEstimation(const Eigen::Matrix4d& start_pose,
                                                                const Eigen::Matrix4d& finish_pose,
                                                                double scan_duration) {
    const Eigen::Matrix4d delta_pose = start_pose.inverse() * finish_pose;
    const Eigen::Vector3d linear_velocity = delta_pose.block<3, 1>(0, 3) / scan_duration;
    const auto angle_axis = Eigen::AngleAxisd(delta_pose.block<3, 3>(0, 0));
    const Eigen::Vector3d angular_velocity = angle_axis.axis() * angle_axis.angle() / scan_duration;
    return std::make_tuple(linear_velocity, angular_velocity);
}

std::vector<Eigen::Vector3d> DeSkewScan(const std::vector<Eigen::Vector3d>& frame,
                                        const std::vector<double>& timestamps,
                                        const Eigen::Vector3d& linear_velocity,
                                        const Eigen::Vector3d& angular_velocity) {
    std::vector<Eigen::Vector3d> corrected_frame(frame.size());
    tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
        const auto& dt = timestamps[i];
        const auto motion = MakeTransform(dt * linear_velocity, dt * angular_velocity);
        corrected_frame[i] = motion * frame[i];
    });
    return corrected_frame;
}

}  // namespace kiss_icp
