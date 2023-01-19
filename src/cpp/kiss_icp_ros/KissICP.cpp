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

#include "KissICP.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

namespace kiss_icp {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d>& frame) {
    // # Apply motion compensation
    // frame = self.compensator.deskew_scan(frame, self.poses, timestamps)

    // Preprocess the input cloud
    const auto& cropped_frame = Preprocess(frame, max_range_, min_range_);

    // Voxelize
    const auto& [source, frame_downsample] = Voxelize(cropped_frame);

    // Get motion prediction and adaptive_threshold
    const double sigma = GetAdaptiveThreshold();

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel();
    const auto last_pose = [&]() -> Eigen::Matrix4d {
        if (poses_.empty()) return Eigen::Matrix4d::Identity();
        return poses_.back();
    }();
    const auto initial_guess = last_pose * prediction;

    // Run icp
    const Eigen::Matrix4d new_pose = local_map_.RegisterPoinCloud(source,         //
                                                                  initial_guess,  //
                                                                  3.0 * sigma,    //
                                                                  sigma / 3.0);
    adaptive_threshold_.UpdateModelDeviation(initial_guess.inverse() * new_pose);
    local_map_.Update(frame_downsample, new_pose);
    poses_.push_back(new_pose);
    return {frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d>& frame) const {
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size_ * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size_ * 1.5);
    return {source, frame_downsample};
}

double KissICP::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return initial_threshold_;
    }
    return adaptive_threshold_.ComputeThreshold();
}

Eigen::Matrix4d KissICP::GetPredictionModel() const {
    const size_t N = poses_.size();
    if (N < 2) return Eigen::Matrix4d::Identity();
    return poses_[N - 2].inverse() * poses_[N - 1];
}

bool KissICP::HasMoved() {
    if (poses_.empty()) return false;
    auto ComputeMotion = [&](const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2) {
        return ((T1.inverse() * T2).block<3, 1>(0, 3)).norm();
    };
    const double motion = ComputeMotion(poses_.front(), poses_.back());
    return motion > 5.0 * min_motion_th_;
}

}  // namespace kiss_icp
