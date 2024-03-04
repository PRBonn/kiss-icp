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
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

#include "VoxelHashMap.hpp"

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

// aliases
using CorrespondenceVector = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

namespace {
inline double square(double x) { return x * x; }

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

// Function to obtain the KNN of one point
Eigen::Vector3d GetClosestNeighborInMap(const Eigen::Vector3d &point,
                                        const kiss_icp::VoxelHashMap &voxel_map) {
    const auto &query_voxels = voxel_map.GetAdjacentVoxels(point);
    const auto &neighbors = voxel_map.GetPoints(query_voxels);
    Eigen::Vector3d closest_neighbor;
    double closest_distance2 = std::numeric_limits<double>::max();
    std::for_each(neighbors.cbegin(), neighbors.cend(), [&](const auto &neighbor) {
        double distance = (neighbor - point).squaredNorm();
        if (distance < closest_distance2) {
            closest_neighbor = neighbor;
            closest_distance2 = distance;
        }
    });

    return closest_neighbor;
}

LinearSystem BuildLinearSystem(const CorrespondenceVector &associations, double kernel) {
    auto compute_jacobian_and_residual = [&](auto i) {
        const auto &[p_source, p_target] = associations[i];
        const Eigen::Vector3d residual = p_source - p_target;
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(p_source);
        return std::make_tuple(J_r, residual);
    };

    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<size_t>{0, associations.size()},
        // Identity
        LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<size_t> &r, LinearSystem J) -> LinearSystem {
            auto Weight = [&](double residual2) {
                return square(kernel) / square(kernel + residual2);
            };
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, residual] = compute_jacobian_and_residual(i);
                const double w = Weight(residual.squaredNorm());
                JTJ_private.noalias() += J_r.transpose() * w * J_r;
                JTr_private.noalias() += J_r.transpose() * w * residual;
            }
            return J;
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        [&](LinearSystem a, const LinearSystem &b) -> LinearSystem {
            a.first += b.first;
            a.second += b.second;
            return a;
        });

    return {JTJ, JTr};
}

CorrespondenceVector GetCorrespondences(const std::vector<Eigen::Vector3d> &points,
                                        const kiss_icp::VoxelHashMap &voxel_map,
                                        double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    CorrespondenceVector points_associated;
    points_associated.reserve(points.size());
    points_associated = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        // Identity
        points_associated,
        // 1st lambda: Parallel computation
        [&](const tbb::blocked_range<points_iterator> &r,
            CorrespondenceVector res) -> CorrespondenceVector {
            res.reserve(r.size());
            for (const auto &point : r) {
                Eigen::Vector3d closest_neighbor = GetClosestNeighborInMap(point, voxel_map);
                if ((closest_neighbor - point).norm() < max_correspondance_distance) {
                    res.emplace_back(point, closest_neighbor);
                }
            }
            return res;
        },
        // 2nd lambda: Parallel reduction
        [](CorrespondenceVector a, const CorrespondenceVector &b) -> CorrespondenceVector {
            a.insert(a.end(),  //
                     std::make_move_iterator(b.cbegin()), std::make_move_iterator(b.cend()));
            return a;
        });

    return points_associated;
}
}  // namespace

namespace kiss_icp {

Sophus::SE3d Registration::AlignCloudToMap(const std::vector<Eigen::Vector3d> &frame,
                                           const VoxelHashMap &voxel_map,
                                           const Sophus::SE3d &initial_guess,
                                           double max_distance,
                                           double kernel) {
    if (voxel_map.Empty()) return initial_guess;

    // Equation (9)
    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        // Equation (10)
        const auto associations = GetCorrespondences(source, voxel_map, max_distance);
        // Equation (11)
        const auto &[JTJ, JTr] = BuildLinearSystem(associations, kernel);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        // Equation (12)
        TransformPoints(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        if (dx.norm() < estimation_threshold_) break;
    }
    // Spit the final transformation
    return T_icp * initial_guess;
}

}  // namespace kiss_icp
