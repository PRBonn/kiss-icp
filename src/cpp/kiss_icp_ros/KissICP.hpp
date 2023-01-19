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

#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

namespace kiss_icp {

class KissICP {
public:
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

public:
    explicit KissICP() {
        // refactor now
        local_map_ = VoxelHashMap(voxel_size_, max_range_, max_points_per_voxel_);
        adaptive_threshold_ = AdaptiveThreshold(initial_threshold_, min_motion_th_, max_range_);
    }

public:
    Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d>& frame);
    Vector3dVectorTuple Voxelize(const std::vector<Eigen::Vector3d>& frame) const;
    double GetAdaptiveThreshold();
    Eigen::Matrix4d GetPredictionModel() const;
    bool HasMoved();

public:
    // Extra C++ API to facilitate ROS debugging
    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); };
    std::vector<Eigen::Matrix4d> poses() const { return poses_; };

private:
    std::vector<Eigen::Matrix4d> poses_;
    // map params
    double voxel_size_ = 1.0;
    double max_range_ = 100.0;
    double min_range_ = 5.0;
    int max_points_per_voxel_ = 20;
    VoxelHashMap local_map_;

    // th parms
    double min_motion_th_ = 0.1;
    double initial_threshold_ = 2.0;
    AdaptiveThreshold adaptive_threshold_;

    // Motion compensation
    bool deksew_ = false;
};

}  // namespace kiss_icp
