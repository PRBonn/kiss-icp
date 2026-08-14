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
#include <array>
#include <numeric>
#include <sophus/se3.hpp>
#include <vector>

#include "VoxelUtils.hpp"

namespace {
using kiss_icp::Voxel;
static const std::array<Voxel, 27> voxel_shifts{
    {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
     Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
     Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
     Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
     Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
     Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};

Eigen::Matrix3d get_lower_bound_for_shift(const Eigen::Vector3d &query, const double voxel_size) {
    const Eigen::Vector3d lower_corner =
        Eigen::Vector3d(std::floor(query.x() / voxel_size), std::floor(query.y() / voxel_size),
                        std::floor(query.z() / voxel_size)) *
        voxel_size;
    const Eigen::Vector3d upper_corner = Eigen::Vector3d(std::floor(query.x() / voxel_size) + 1.0,
                                                         std::floor(query.y() / voxel_size) + 1.0,
                                                         std::floor(query.z() / voxel_size) + 1.0) *
                                         voxel_size;
    Eigen::Matrix3d lower_bounds;
    lower_bounds.col(0) = (query - lower_corner).array().square();
    lower_bounds.col(1).setZero();
    lower_bounds.col(2) = (upper_corner - query).array().square();
    return lower_bounds;
}
}  // namespace

namespace kiss_icp {
void VoxelBlock::emplace_back(const Eigen::Vector3d &p) {
    if (num_points < max_points_per_voxel) {
        points[num_points++] = p;
    }
}

std::optional<Eigen::Vector3d> VoxelHashMap::GetClosestNeighbor(const Eigen::Vector3d &query,
                                                                const double max_distance) const {
    // Convert the point to voxel coordinates
    bool found_neighbor = false;
    const auto voxel = PointToVoxel(query, voxel_size_);
    const auto lower_bounds = get_lower_bound_for_shift(query, voxel_size_);

    // Find the nearest neighbor
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance2 = max_distance * max_distance;

    for (const auto &voxel_shift : voxel_shifts) {
        const double lower_bound2 = lower_bounds(0, voxel_shift.x() + 1) +
                                    lower_bounds(1, voxel_shift.y() + 1) +
                                    lower_bounds(2, voxel_shift.z() + 1);
        if (lower_bound2 >= closest_distance2) {
            continue;
        }

        auto search = map_.find(voxel + voxel_shift);
        if (search == map_.end()) continue;

        const auto &points = search.value();
        for (const auto &point : points) {
            const double distance2 = (point - query).squaredNorm();
            if (distance2 < closest_distance2) {
                closest_neighbor = point;
                closest_distance2 = distance2;
                found_neighbor = true;
            }
        }
    }
    if (found_neighbor) {
        return closest_neighbor;
    }
    return std::nullopt;
}

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(map_.size() * static_cast<size_t>(max_points_per_voxel));
    for (const auto &map_element : map_) {
        const auto &voxel_points = map_element.second;
        points.insert(points.end(), voxel_points.cbegin(), voxel_points.cend());
    }
    return points;
}

size_t VoxelHashMap::Size() const {
    size_t total_points = 0;
    for (const auto &map_element : map_) {
        total_points += map_element.second.size();
    }
    return total_points;
}

void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points,
                          const Eigen::Vector3d &origin) {
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
}

void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    const Eigen::Matrix3d R = pose.rotationMatrix();
    const Eigen::Vector3d t = pose.translation();
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return R * point + t; });
    Update(points_transformed, t);
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    for (const auto &point : points) {
        const auto voxel = PointToVoxel(point, voxel_size_);
        VoxelBlock &voxel_points = map_[voxel];
        if (voxel_points.size() == max_points_per_voxel ||
            std::any_of(voxel_points.cbegin(), voxel_points.cend(), [&](const auto &voxel_point) {
                return (voxel_point - point).squaredNorm() < map_resolution2_;
            })) {
            continue;
        }
        voxel_points.emplace_back(point);
    }
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    const auto max_distance2 = max_distance_ * max_distance_;
    for (auto it = map_.begin(); it != map_.end();) {
        it = (VoxelCenter(it->first, voxel_size_) - origin).squaredNorm() >= max_distance2
                 ? map_.erase(it)
                 : std::next(it);
    }
}
}  // namespace kiss_icp
