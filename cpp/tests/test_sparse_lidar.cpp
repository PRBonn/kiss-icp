#include <Eigen/Core>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "kiss_icp/core/VoxelUtils.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"

namespace {
using kiss_icp::pipeline::KISSConfig;
using kiss_icp::pipeline::KissICP;
using Points = std::vector<Eigen::Vector3d>;

void Require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

bool SamePoints(Points lhs, Points rhs) {
    const auto less = [](const auto &a, const auto &b) {
        return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3);
    };
    std::sort(lhs.begin(), lhs.end(), less);
    std::sort(rhs.begin(), rhs.end(), less);
    return lhs.size() == rhs.size() &&
           std::equal(lhs.begin(), lhs.end(), rhs.begin(),
                      [](const auto &a, const auto &b) { return (a.array() == b.array()).all(); });
}

void TestSparseLidar() {
    // The first two points share a fine voxel; the middle two share a coarse voxel.
    const Points frame{
        {1.1, 1.1, 1.1}, {1.2, 1.2, 1.2}, {1.6, 1.1, 1.1}, {2.1, 1.1, 1.1}, {3.1, 1.1, 1.1}};
    KISSConfig config;
    config.deskew = false;
    KissICP standard(config);
    const auto [source, map_points] = standard.Voxelize(frame);
    const auto expected_map = kiss_icp::VoxelDownsample(frame, config.voxel_size * 0.5);
    const auto expected_source = kiss_icp::VoxelDownsample(expected_map, config.voxel_size * 1.5);
    Require(source.size() == 3 && map_points.size() == 4, "Unexpected test cloud voxel counts");
    Require(SamePoints(source, expected_source), "Default registration downsampling changed");
    Require(SamePoints(map_points, expected_map), "Default map downsampling changed");

    config.sparse_lidar = true;
    KissICP sparse(config);
    const auto [sparse_source, sparse_map] = sparse.Voxelize(frame);
    Require(SamePoints(sparse_source, expected_map), "Sparse mode discarded first-pass points");
    Require(SamePoints(sparse_map, map_points), "Sparse mode changed map downsampling");

    // With an empty map, both pipelines keep the initial pose and insert the same points.
    standard.RegisterFrame(frame, {});
    sparse.RegisterFrame(frame, {});
    Require(SamePoints(standard.LocalMap(), sparse.LocalMap()), "Initial map update changed");

    for (const auto *pipeline : {&standard, &sparse}) {
        const auto [empty_source, empty_map] = pipeline->Voxelize({});
        Require(empty_source.empty() && empty_map.empty(), "Empty scan produced points");
    }
}
}  // namespace

int main() {
    try {
        TestSparseLidar();
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
