#include "VoxelUtils.hpp"

#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <vector>

namespace kiss_icp {

std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             const double voxel_size) {
    tsl::robin_map<Voxel, Eigen::Vector3d, VoxelHash> grid;
    grid.reserve(frame.size());
    for (const auto &point : frame) {
        const auto voxel = PointToVoxel(point, voxel_size);
        if (!grid.contains(voxel)) grid.try_emplace(voxel, point);
    }
    std::vector<Eigen::Vector3d> frame_dowsampled;
    frame_dowsampled.reserve(grid.size());
    for (const auto &grid_element : grid) {
        frame_dowsampled.emplace_back(grid_element.second);
    }
    return frame_dowsampled;
}

}  // namespace kiss_icp
