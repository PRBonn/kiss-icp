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
#include "VoxelHashMap.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <vector>

namespace kiss_icp {

std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             double voxel_size) {
    tsl::robin_map<Voxel, Eigen::Vector3d> grid;
    grid.reserve(frame.size());
    for (const auto &point : frame) {
        const auto voxel = Voxel(point, voxel_size);
        if (grid.contains(voxel)) continue;
        grid.insert({voxel, point});
    }
    std::vector<Eigen::Vector3d> frame_dowsampled;
    frame_dowsampled.reserve(grid.size());
    for (const auto &[_, point] : grid) {
        frame_dowsampled.emplace_back(point);
    }
    return frame_dowsampled;
}

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(max_points_per_voxel_ * map_.size());
    for (const auto &[voxel, voxel_block] : map_) {
        std::for_each(voxel_block.points.cbegin(), voxel_block.points.cend(),
                      [&](const auto &point) { points.emplace_back(point); });
    }
    return points;
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points,
                             const Eigen::Vector3d &origin) {
    // Insert points to map
    auto AddPoint = [&](const auto &point) {
        auto voxel = Voxel(point, voxel_size_);
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_block = search.value();
            voxel_block.add_point(point);
        } else {
            map_.insert({voxel, VoxelBlock(point, max_points_per_voxel_)});
        }
    };

    // Clear the clutter
    auto RemovePointsFarFromLocation = [&]() {
        for (const auto &[voxel, voxel_block] : map_) {
            const Eigen::Vector3d &pt = voxel_block.points.front();
            if ((pt - origin).squaredNorm() > (max_distance_ * max_distance_)) {
                map_.erase(voxel);
            }
        }
    };

    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) { AddPoint(point); });
    RemovePointsFarFromLocation();
}

// Lambda Function to obtain the KNN of one point, maybe refactor
Eigen::Vector3d VoxelHashMap::GetClosestNeighboor(const Eigen::Vector3d &point) const {
    auto kx = static_cast<int>(point[0] / voxel_size_);
    auto ky = static_cast<int>(point[1] / voxel_size_);
    auto kz = static_cast<int>(point[2] / voxel_size_);
    std::vector<Voxel> voxels;
    voxels.reserve(27);
    for (int i = kx - 1; i < kx + 1 + 1; ++i) {
        for (int j = ky - 1; j < ky + 1 + 1; ++j) {
            for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                voxels.emplace_back(i, j, k);
            }
        }
    }

    using Vector3dVector = std::vector<Eigen::Vector3d>;
    Vector3dVector neighboors;
    neighboors.reserve(27 * max_points_per_voxel_);
    std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            const auto &points = search->second.points;
            if (!points.empty()) {
                for (const auto &point : points) {
                    neighboors.emplace_back(point);
                }
            }
        }
    });

    Eigen::Vector3d closest_neighbor;
    double closest_distance2 = std::numeric_limits<double>::max();
    std::for_each(neighboors.cbegin(), neighboors.cend(), [&](const auto &neighbor) {
        double distance = (neighbor - point).squaredNorm();
        if (distance < closest_distance2) {
            closest_neighbor = neighbor;
            closest_distance2 = distance;
        }
    });

    return closest_neighbor;
};

}  // namespace kiss_icp
