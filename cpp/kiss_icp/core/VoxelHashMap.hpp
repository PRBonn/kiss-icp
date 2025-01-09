// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once
#include <Eigen/Core>
#include <array>
#include <bonxai/bonxai.hpp>
#include <sophus/se3.hpp>

namespace kiss_icp {

using VoxelBlock = std::vector<Eigen::Vector3d>;

struct VoxelHashMap {
    explicit VoxelHashMap(const double voxel_size,
                          const double clipping_distance,
                          const unsigned int max_points_per_voxel);

    inline void Clear() { map_.clear(Bonxai::ClearOption::CLEAR_MEMORY); }
    inline bool Empty() const { return map_.activeCellsCount() == 0; }
    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    void AddPoints(const std::vector<Eigen::Vector3d> &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const Eigen::Vector3d &query) const;

    double voxel_size_;
    double clipping_distance_;
    unsigned int max_points_per_voxel_;
    Bonxai::VoxelGrid<VoxelBlock> map_;

private:
    using AccessorType = typename Bonxai::VoxelGrid<VoxelBlock>::Accessor;
    AccessorType accessor_;
};

}  // namespace kiss_icp
