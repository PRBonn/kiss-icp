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
#include <algorithm>
#include <tuple>
#include <vector>

#include "Transforms.hpp"

namespace {
using VelocityTuple = std::tuple<Eigen::Vector3d, Eigen::Vector3d>;
VelocityTuple VelocityEstimation(const Eigen::Isometry3d &start_pose,
                                 const Eigen::Isometry3d &finish_pose,
                                 double scan_duration) {
    const auto delta_pose = kiss_icp::ConcatenateIsometries(start_pose.inverse(), finish_pose);
    const auto linear_velocity = delta_pose.translation() / scan_duration;
    const auto rotation = Eigen::AngleAxisd(delta_pose.rotation());
    const auto angular_velocity = rotation.axis() * rotation.angle() / scan_duration;
    return std::make_tuple(linear_velocity, angular_velocity);
}
}  // namespace

namespace kiss_icp {

using Vector3dVector = std::vector<Eigen::Vector3d>;
Vector3dVector MotionCompensator::DeSkewScan(const std::vector<Eigen::Vector3d> &frame,
                                             const std::vector<double> &timestamps,
                                             const Eigen::Isometry3d &start_pose,
                                             const Eigen::Isometry3d &finish_pose) {
    // Estimate linear and angular velocities
    const auto [lvel, avel] = VelocityEstimation(start_pose, finish_pose, scan_duration_);

    // TODO(Nacho) Add explanation of this
    std::vector<double> timestamps_ = timestamps;
    std::for_each(timestamps_.begin(), timestamps_.end(), [&](double &timestamp) {
        timestamp = scan_duration_ * (timestamp - mid_pose_timestamp_);
    });

    std::vector<Eigen::Vector3d> corrected_frame(frame.size());
    tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
        const auto &dt = timestamps_[i];
        const auto motion = MakeTransform(dt * lvel, dt * avel);
        corrected_frame[i] = motion * frame[i];
    });
    return corrected_frame;
}

}  // namespace kiss_icp
