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
#include <optional>
#include <sophus/se3.hpp>
#include <vector>

#include "VoxelUtils.hpp"

namespace kiss_icp {

static constexpr unsigned int max_points_per_voxel = 20;

struct VoxelBlock {
    void emplace_back(const Eigen::Vector3d &point);
    size_t size() const { return num_points; }
    auto begin() const { return points.begin(); }
    auto cbegin() const { return points.cbegin(); }
    auto end() const { return std::next(points.begin(), num_points); }
    auto cend() const { return std::next(points.cbegin(), num_points); }
    Eigen::Vector3d front() const { return points.front(); }
    std::array<Eigen::Vector3d, max_points_per_voxel> points;
    size_t num_points = 0;
};

struct VoxelHashMap {
    explicit VoxelHashMap(double voxel_size, double max_distance)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          map_resolution2_(voxel_size * voxel_size / max_points_per_voxel) {}
    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }
    size_t Size() const;
    void Update(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &origin);
    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    void AddPoints(const std::vector<Eigen::Vector3d> &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    std::optional<Eigen::Vector3d> GetClosestNeighbor(const Eigen::Vector3d &query,
                                                      const double max_distance) const;

    double voxel_size_;
    double max_distance_;
    double map_resolution2_;
    tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map_;
};
}  // namespace kiss_icp
