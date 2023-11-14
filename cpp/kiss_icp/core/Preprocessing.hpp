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
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

namespace kiss_icp {

/// Crop the frame with max/min ranges
std::vector<Eigen::Vector3d> Preprocess(const std::vector<Eigen::Vector3d> &frame,
                                        double max_range,
                                        double min_range);

/// This function only applies for the KITTI dataset, and should NOT be used by any other dataset,
/// the original idea and part of the implementation is taking from CT-ICP(Although IMLS-SLAM
/// Originally introduced the calibration factor)
std::vector<Eigen::Vector3d> CorrectKITTIScan(const std::vector<Eigen::Vector3d> &frame);

/// Voxelize point cloud keeping the original coordinates
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             double voxel_size);
}  // namespace kiss_icp
