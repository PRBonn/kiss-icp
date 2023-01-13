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
//
// NOTE: This implementation is heavily inspired in the original CT-ICP VoxelHashMap implementation,
// although it was heavily modifed and drastically simplified, but if you are using this module you
// should at least acknoowledge the work from CT-ICP by giving a star on GitHub

#pragma once

#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <queue>
#include <vector>

namespace kiss_icp {

struct Voxel {
    Voxel(int32_t x, int32_t y, int32_t z) : ijk({x, y, z}) {}
    Voxel(const Eigen::Vector3d &point, double voxel_size)
        : ijk({static_cast<int32_t>(point.x() / voxel_size),
               static_cast<int32_t>(point.y() / voxel_size),
               static_cast<int32_t>(point.z() / voxel_size)}) {}
    bool operator==(const Voxel &vox) const {
        return ijk[0] == vox.ijk[0] && ijk[1] == vox.ijk[1] && ijk[2] == vox.ijk[2];
    }

    std::array<int32_t, 3> ijk;
};

struct VoxelBlock {
    explicit VoxelBlock(int num_points) : num_points_(num_points) { points.reserve(num_points); }

    explicit VoxelBlock(const Eigen::Vector3d &point, int num_points) : num_points_(num_points) {
        points.reserve(num_points);
        points.emplace_back(point);
    }

    void add_point(const Eigen::Vector3d &point) {
        if (IsFull()) return;
        points.emplace_back(point);
    }

    inline bool IsFull() const { return static_cast<size_t>(num_points_) == points.size(); }
    inline size_t size() const { return points.size(); }

    std::vector<Eigen::Vector3d> points;
    int num_points_{};
};

}  // namespace kiss_icp

// Specialization of std::hash for our custom type Voxel
namespace std {

template <>
struct hash<kiss_icp::Voxel> {
    std::size_t operator()(const kiss_icp::Voxel &vox) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(vox.ijk.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
}  // namespace std

namespace kiss_icp {
class VoxelHashMap {
public:
    explicit VoxelHashMap(double voxel_size, double max_distance, int max_points_per_voxel)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel) {}

public:
    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }
    void AddPoints(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    Eigen::Vector3d GetClosestNeighboor(const Eigen::Vector3d &point) const;

private:
    // mapping parameters
    double voxel_size_;
    double max_distance_;
    int max_points_per_voxel_;
    tsl::robin_map<Voxel, VoxelBlock> map_;
};

std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             double voxel_size);

}  // namespace kiss_icp
