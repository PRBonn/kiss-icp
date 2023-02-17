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
#include <vector>

namespace kiss_icp {

class MotionCompensator {
public:
    /// Constructs a Odometry-based motion compensation scheme
    explicit MotionCompensator(double frame_rate) : scan_duration_(1 / frame_rate) {}

    /// Compensate the frame by estimatng the velocity between the given poses
    std::vector<Eigen::Vector3d> DeSkewScan(const std::vector<Eigen::Vector3d> &frame,
                                            const std::vector<double> &timestamps,
                                            const Eigen::Isometry3d &start_pose,
                                            const Eigen::Isometry3d &finish_pose);

private:
    double scan_duration_;
    double mid_pose_timestamp_{0.5};
};

}  // namespace kiss_icp
