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

#include "VoxelHashMap.hpp"

#include <Eigen/Core>
#include <array>
#include <bonxai/bonxai.hpp>
#include <cstdint>
#include <sophus/se3.hpp>

#include "bonxai/grid_coord.hpp"

namespace {
static constexpr std::array<Bonxai::CoordT, 27> shifts{
    Bonxai::CoordT{-1, -1, -1}, Bonxai::CoordT{-1, -1, 0}, Bonxai::CoordT{-1, -1, 1},
    Bonxai::CoordT{-1, 0, -1},  Bonxai::CoordT{-1, 0, 0},  Bonxai::CoordT{-1, 0, 1},
    Bonxai::CoordT{-1, 1, -1},  Bonxai::CoordT{-1, 1, 0},  Bonxai::CoordT{-1, 1, 1},

    Bonxai::CoordT{0, -1, -1},  Bonxai::CoordT{0, -1, 0},  Bonxai::CoordT{0, -1, 1},
    Bonxai::CoordT{0, 0, -1},   Bonxai::CoordT{0, 0, 0},   Bonxai::CoordT{0, 0, 1},
    Bonxai::CoordT{0, 1, -1},   Bonxai::CoordT{0, 1, 0},   Bonxai::CoordT{0, 1, 1},

    Bonxai::CoordT{1, -1, -1},  Bonxai::CoordT{1, -1, 0},  Bonxai::CoordT{1, -1, 1},
    Bonxai::CoordT{1, 0, -1},   Bonxai::CoordT{1, 0, 0},   Bonxai::CoordT{1, 0, 1},
    Bonxai::CoordT{1, 1, -1},   Bonxai::CoordT{1, 1, 0},   Bonxai::CoordT{1, 1, 1}};

static constexpr uint8_t inner_grid_log2_dim = 2;
static constexpr uint8_t leaf_grid_log2_dim = 3;
}  // namespace

namespace kiss_icp {

VoxelHashMap::VoxelHashMap(const double voxel_size,
                           const double clipping_distance,
                           const unsigned int max_points_per_voxel)
    : voxel_size_(voxel_size),
      clipping_distance_(clipping_distance),
      max_points_per_voxel_(max_points_per_voxel),
      map_(voxel_size, inner_grid_log2_dim, leaf_grid_log2_dim),
      accessor_(map_.createAccessor()) {}

std::tuple<Eigen::Vector3d, double> VoxelHashMap::GetClosestNeighbor(
    const Eigen::Vector3d &query) const {
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance = std::numeric_limits<double>::max();
    const auto const_accessor = map_.createConstAccessor();
    const Bonxai::CoordT voxel = map_.posToCoord(query);
    std::for_each(shifts.cbegin(), shifts.cend(), [&](const Bonxai::CoordT &voxel_shift) {
        const Bonxai::CoordT query_voxel = voxel + voxel_shift;
        const VoxelBlock *voxel_points = const_accessor.value(query_voxel);
        if (voxel_points != nullptr) {
            const Eigen::Vector3d &neighbor =
                *std::min_element(voxel_points->cbegin(), voxel_points->cend(),
                                  [&](const auto &lhs, const auto &rhs) {
                                      return (lhs - query).norm() < (rhs - query).norm();
                                  });
            double distance = (neighbor - query).norm();
            if (distance < closest_distance) {
                closest_neighbor = neighbor;
                closest_distance = distance;
            }
        }
    });
    return std::make_tuple(closest_neighbor, closest_distance);
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
    std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector3d &p) {
        const auto voxel_coordinates = map_.posToCoord(p);
        VoxelBlock *voxel_points = accessor_.value(voxel_coordinates, /*create_if_missing=*/true);
        if (voxel_points->size() == max_points_per_voxel_ ||
            std::any_of(voxel_points->cbegin(), voxel_points->cend(), [&](const auto &voxel_point) {
                return (voxel_point - p).norm() < map_resolution;
            })) {
            return;
        }
        voxel_points->reserve(max_points_per_voxel_);
        voxel_points->emplace_back(p);
    });
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    auto is_too_far_away = [&](const VoxelBlock &block) {
        return (block.front() - origin).norm() > clipping_distance_;
    };

    std::vector<Bonxai::CoordT> keys_to_delete;
    auto &root_map = map_.rootMap();
    for (auto &[key, inner_grid] : root_map) {
        for (auto inner_it = inner_grid.mask().beginOn(); inner_it; ++inner_it) {
            const int32_t inner_index = *inner_it;
            auto &leaf_grid = inner_grid.cell(inner_index);
            const auto &voxel_block = leaf_grid->cell(leaf_grid->mask().findFirstOn());
            if (is_too_far_away(voxel_block)) {
                inner_grid.mask().setOff(inner_index);
                leaf_grid.reset();
            }
        }
        if (inner_grid.mask().isOff()) {
            keys_to_delete.push_back(key);
        }
    }
    for (const auto &key : keys_to_delete) {
        root_map.erase(key);
    }
}

void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    AddPoints(points_transformed);
    RemovePointsFarFromLocation(origin);
}

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> point_cloud;
    point_cloud.reserve(map_.activeCellsCount() * max_points_per_voxel_);
    map_.forEachCell([&point_cloud, this](const VoxelBlock &block, const auto &) {
        point_cloud.insert(point_cloud.end(), block.cbegin(), block.cend());
    });
    return point_cloud;
}

}  // namespace kiss_icp
