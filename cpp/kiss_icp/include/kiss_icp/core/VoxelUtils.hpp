// MIT License
//
// Copyright (c) 2024 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
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
//
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace kiss_icp {

using Voxel = Eigen::Vector3i;
inline Voxel PointToVoxel(const Eigen::Vector3d &point, const double voxel_size) {
    return Voxel(static_cast<int>(std::floor(point.x() / voxel_size)),
                 static_cast<int>(std::floor(point.y() / voxel_size)),
                 static_cast<int>(std::floor(point.z() / voxel_size)));
}

/// Voxelize a point cloud keeping the original coordinates
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             const double voxel_size);

}  // namespace kiss_icp

template <>
struct std::hash<kiss_icp::Voxel> {
    std::size_t operator()(const kiss_icp::Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};
