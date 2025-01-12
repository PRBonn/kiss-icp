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
#include <iostream>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

namespace {
using namespace kiss_icp;
using StampedPointCloud = std::tuple<std::vector<Eigen::Vector3d>, std::vector<double>>;
StampedPointCloud Downsample(const std::vector<Eigen::Vector3d> &frame,
                             const std::vector<double> &timestamps,
                             const double voxel_size,
                             const double max_range,
                             const double min_range) {
    auto is_in_range = [&](const auto &point) {
        const double point_range = point.norm();
        return point_range < max_range && point_range > min_range;
    };
    tsl::robin_map<Voxel, size_t> grid;
    grid.reserve(frame.size());
    for (size_t i = 0; i < frame.size(); ++i) {
        const auto &point = frame[i];
        const auto voxel = PointToVoxel(point, voxel_size);
        if (!grid.contains(voxel) && is_in_range(point)) {
            grid.insert({voxel, i});
        };
    };
    std::vector<Eigen::Vector3d> frame_dowsampled;
    frame_dowsampled.reserve(grid.size());
    std::vector<double> timestamps_dowsampled;
    timestamps_dowsampled.reserve(grid.size());
    std::for_each(grid.cbegin(), grid.cend(), [&](const auto &voxel_and_point) {
        const auto &index = voxel_and_point.second;
        frame_dowsampled.emplace_back(frame[index]);
        timestamps_dowsampled.emplace_back(timestamps[index]);
    });
    return std::make_tuple(frame_dowsampled, timestamps_dowsampled);
}
std::tuple<StampedPointCloud, StampedPointCloud> Preprocess(
    const std::vector<Eigen::Vector3d> &frame,
    const std::vector<double> &timestamps,
    const double voxel_size,
    const double max_range,
    const double min_range,
    const bool deskew) {
    const std::vector<double> &stamps = std::invoke([&]() {
        if (timestamps.empty() || !deskew) {
            return std::vector<double>(frame.size(), 1.0);
        }
        return timestamps;
    });
    const auto &[pts_downsampled, stamps_downsampled] =
        Downsample(frame, stamps, 0.5 * voxel_size, max_range, min_range);
    const auto &[pts_source, stamps_source] =
        Downsample(pts_downsampled, stamps_downsampled, 1.5 * voxel_size, max_range, min_range);
    return std::make_tuple(std::make_tuple(pts_source, stamps_source),
                           std::make_tuple(pts_downsampled, stamps_downsampled));
}

std::vector<Eigen::Vector3d> DeskewScan(const StampedPointCloud &frame, const State &state) {
    const auto &[pts, stamps] = frame;
    std::vector<Eigen::Vector3d> deskewed_frame(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        deskewed_frame[i] = state.transformPoint(pts[i], stamps[i]);
    }
    return deskewed_frame;
}
}  // namespace

namespace kiss_icp::pipeline {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                    const std::vector<double> &timestamps) {
    // Preprocess the input cloud
    const auto &[pcd_source, pcd_downsampled] =
        Preprocess(frame, timestamps, config_.voxel_size, config_.max_range, config_.min_range,
                   config_.deskew);

    const auto &[source, source_stamps] = pcd_source;

    // Get adaptive_threshold
    const double sigma = adaptive_threshold_.ComputeThreshold();

    const auto previous_pose = state_.poseAtNormalizedTime(1.0);

    // Run ICP
    state_ = registration_.AlignPointsToMap(source,  // frame
                                            source_stamps,
                                            local_map_,    // voxel_map
                                            state_,        // initial_guess
                                            3.0 * sigma,   // max_correspondence_dist
                                            sigma / 3.0);  // kernel

    // Compute the difference between the prediction and the actual estimate
    const auto new_pose = state_.poseAtNormalizedTime(1.0);
    const auto model_deviation = previous_pose.inverse() * new_pose;

    // Update step: threshold, local map, delta, and the last pose
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    const auto &frame_downsampled = DeskewScan(pcd_downsampled, state_);
    local_map_.Update(frame_downsampled, new_pose.translation());
    last_delta_ = last_pose_.inverse() * new_pose;
    last_pose_ = new_pose;
    state_.computeNextState();

    // Return the (deskew) input raw scan (frame) and the points used for registration (source)
    return {frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}

}  // namespace kiss_icp::pipeline
