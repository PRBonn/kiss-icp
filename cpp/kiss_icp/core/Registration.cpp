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
#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

#include "VoxelHashMap.hpp"

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

using Correspondence = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
using Associations = std::vector<Correspondence>;
using LinearSystem = std::tuple<Eigen::Matrix6d, Eigen::Vector6d, double>;

namespace {
inline double square(double x) { return x * x; }

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

using Voxel = kiss_icp::VoxelHashMap::Voxel;
std::vector<Voxel> GetAdjacentVoxels(const Voxel &voxel, int adjacent_voxels = 1) {
    std::vector<Voxel> voxel_neighborhood;
    for (int i = voxel.x() - adjacent_voxels; i < voxel.x() + adjacent_voxels + 1; ++i) {
        for (int j = voxel.y() - adjacent_voxels; j < voxel.y() + adjacent_voxels + 1; ++j) {
            for (int k = voxel.z() - adjacent_voxels; k < voxel.z() + adjacent_voxels + 1; ++k) {
                voxel_neighborhood.emplace_back(i, j, k);
            }
        }
    }
    return voxel_neighborhood;
}

std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const Eigen::Vector3d &point,
                                                       const kiss_icp::VoxelHashMap &voxel_map) {
    // Convert the point to voxel coordinates
    const auto &voxel = voxel_map.PointToVoxel(point);
    // Get nearby voxels on the map
    const auto &query_voxels = GetAdjacentVoxels(voxel);
    // Extract the points contained within the neighborhood voxels
    const auto &neighbors = voxel_map.GetPoints(query_voxels);

    // Find the nearest neighbor
    Eigen::Vector3d closest_neighbor;
    double closest_distance = std::numeric_limits<double>::max();
    std::for_each(neighbors.cbegin(), neighbors.cend(), [&](const auto &neighbor) {
        double distance = (neighbor - point).norm();
        if (distance < closest_distance) {
            closest_neighbor = neighbor;
            closest_distance = distance;
        }
    });
    return std::make_tuple(closest_neighbor, closest_distance);
}

Associations FindAssociations(const std::vector<Eigen::Vector3d> &points,
                              const kiss_icp::VoxelHashMap &voxel_map,
                              double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Associations associations;
    associations.reserve(points.size());
    associations = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        // Identity
        associations,
        // 1st lambda: Parallel computation
        [&](const tbb::blocked_range<points_iterator> &r, Associations res) -> Associations {
            res.reserve(r.size());
            std::for_each(r.begin(), r.end(), [&](const auto &point) {
                const auto &[closest_neighbor, distance] = GetClosestNeighbor(point, voxel_map);
                if (distance < max_correspondance_distance) {
                    res.emplace_back(point, closest_neighbor);
                }
            });
            return res;
        },
        // 2nd lambda: Parallel reduction
        [](Associations a, const Associations &b) -> Associations {
            a.insert(a.end(),                              //
                     std::make_move_iterator(b.cbegin()),  //
                     std::make_move_iterator(b.cend()));
            return a;
        });

    return associations;
}

LinearSystem BuildLinearSystem(const Associations &associations,
                               std::function<double_t(double_t)> kernel) {
    auto compute_jacobian_and_residual = [](auto association) {
        const auto &[source, target] = association;
        const Eigen::Vector3d residual = source - target;
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
        return std::make_tuple(J_r, residual);
    };

    auto sum_linear_systems = [](LinearSystem a, const LinearSystem &b) {
        std::get<Eigen::Matrix6d>(a) += std::get<Eigen::Matrix6d>(b);
        std::get<Eigen::Vector6d>(a) += std::get<Eigen::Vector6d>(b);
        std::get<double>(a) += std::get<double>(b);
        return a;
    };

    using associations_iterator = Associations::const_iterator;
    const auto &[JTJ, JTr, chi_square] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<associations_iterator>{associations.cbegin(), associations.cend()},
        // Identity
        LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero(), 0.0),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<associations_iterator> &r, LinearSystem J) -> LinearSystem {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &association) {
                    const auto &[J_r, residual] = compute_jacobian_and_residual(association);
                    const double chi_square = residual.squaredNorm();
                    const double w = kernel(chi_square);
                    return LinearSystem(J_r.transpose() * w * J_r,                    // JTJ
                                        J_r.transpose() * w * residual, chi_square);  // JTr
                });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);

    return {JTJ, JTr, chi_square};
}
}  // namespace

namespace kiss_icp {

Registration::Registration(int max_num_iteration, double convergence_criterion, int max_num_threads)
    : max_num_iterations_(max_num_iteration),
      convergence_criterion_(convergence_criterion),
      // Only manipulate the number of threads if the user specifies something greater than 0
      max_num_threads_(max_num_threads > 0 ? max_num_threads : tbb::info::default_concurrency()) {
    // This global variable requires static duration storage to be able to manipulate the max
    // concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

Estimate Registration::AlignPointsToMap(const std::vector<Eigen::Vector3d> &frame,
                                        const VoxelHashMap &voxel_map,
                                        const Estimate &initial_guess,
                                        double max_correspondence_distance,
                                        double kernel_threshold) {
    if (voxel_map.Empty()) return initial_guess;

    auto GM = [&](double residual2) {
        return square(kernel_threshold) / square(kernel_threshold + residual2);
    };
    // Equation (9)
    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess.pose, source);

    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        // Equation (10)
        const auto associations = FindAssociations(source, voxel_map, max_correspondence_distance);
        // Equation (11)
        const auto &[JTJ, JTr, chi_square] = BuildLinearSystem(associations, GM);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        // Equation (12)
        TransformPoints(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        if (dx.norm() < convergence_criterion_) break;
    }
    const auto associations = FindAssociations(source, voxel_map, max_correspondence_distance);
    const auto &[H, b, chi_square] =
        BuildLinearSystem(associations, [](double x) { return x / x; });
    const auto icp_covariance =
        (chi_square / static_cast<double>(associations.size()) * H).inverse();
    Estimate estimate;
    estimate.pose = T_icp * initial_guess.pose;
    const auto A = T_icp.inverse().Adj();
    estimate.covariance = A * initial_guess.covariance * A.transpose() + icp_covariance;
    // Spit the final transformation
    return estimate;
}

}  // namespace kiss_icp
