// MIT License
//
// Copyright (c) 2025 Tiziano Guadagnino
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
#include "ContinousRegistration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

#include "State.hpp"
#include "VoxelHashMap.hpp"
#include "VoxelUtils.hpp"

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

using Association = std::tuple<Eigen::Vector3d, double, Eigen::Vector3d>;
using Correspondences = tbb::concurrent_vector<Association>;
using LinearSystem = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

namespace {
inline double square(const double x) { return x * x; }
inline double cube(const double x) { return square(x) * x; }

Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const std::vector<double> &stamps,
                                const kiss_icp::State &x,
                                const kiss_icp::VoxelHashMap &voxel_map,
                                const double max_correspondance_distance) {
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<size_t>{0, points.size()}, [&](const tbb::blocked_range<size_t> &r) {
            for (size_t idx = r.begin(); idx < r.end(); ++idx) {
                const auto &point = points[idx];
                const auto &stamp = stamps[idx];
                const auto &[closest_neighbor, distance] =
                    voxel_map.GetClosestNeighbor(x.poseAtNormalizedTime(stamp) * point);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(point, stamp, closest_neighbor);
                }
            }
        });
    return correspondences;
}

LinearSystem BuildLinearSystem(const Correspondences &correspondences,
                               const kiss_icp::State &x,
                               const double kernel_scale) {
    auto compute_jacobian_and_residual = [&](const auto &correspondence) {
        const auto &[source, alpha, target] = correspondence;
        const Sophus::SE3d &pose = x.poseAtNormalizedTime(alpha);
        const Eigen::Vector3d &transformed_point = pose * source;
        const Eigen::Vector3d &residual = transformed_point - target;
        const Eigen::Matrix3d &R = pose.so3().matrix();
        const double time = 1.0 / 6.0 * cube(alpha);
        Eigen::Matrix3_6d J_icp;
        J_icp.block<3, 3>(0, 0) = R * time;
        J_icp.block<3, 3>(0, 3) = -R * Sophus::SO3d::hat(source) * time;
        return std::make_tuple(J_icp, residual);
    };

    auto sum_linear_systems = [](LinearSystem a, const LinearSystem &b) {
        a.first += b.first;
        a.second += b.second;
        return a;
    };

    auto GM_weight = [&](const double &residual2) {
        return square(kernel_scale) / square(kernel_scale + residual2);
    };

    using correspondence_iterator = Correspondences::const_iterator;
    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<correspondence_iterator>{correspondences.cbegin(),
                                                    correspondences.cend()},
        // Identity
        LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero()),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<correspondence_iterator> &r, LinearSystem J) -> LinearSystem {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &correspondence) {
                    const auto &[J_r, residual] = compute_jacobian_and_residual(correspondence);
                    const double w = GM_weight(residual.squaredNorm());
                    return LinearSystem(J_r.transpose() * w * J_r,        // JTJ
                                        J_r.transpose() * w * residual);  // JTr
                });
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        sum_linear_systems);

    return {JTJ, JTr};
}
}  // namespace

namespace kiss_icp {

ContinousRegistration::ContinousRegistration(int max_num_iteration,
                                             double convergence_criterion,
                                             int max_num_threads)
    : max_num_iterations_(max_num_iteration),
      convergence_criterion_(convergence_criterion),
      // Only manipulate the number of threads if the user specifies something greater than 0
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : tbb::this_task_arena::max_concurrency()) {
    // This global variable requires static duration storage to be able to manipulate the max
    // concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

State ContinousRegistration::AlignPointsToMap(const std::vector<Eigen::Vector3d> &frame,
                                              const std::vector<double> &timestamps,
                                              const VoxelHashMap &voxel_map,
                                              const State &initial_guess,
                                              const double max_distance,
                                              const double kernel_scale) {
    if (voxel_map.Empty()) return initial_guess;

    // Equation (9)
    std::vector<Eigen::Vector3d> source = frame;
    State x = initial_guess;
    for (int j = 0; j < max_num_iterations_; ++j) {
        // Equation (10)
        const auto correspondences =
            DataAssociation(source, timestamps, x, voxel_map, max_distance);
        // Equation (11)
        const auto &[JTJ, JTr] = BuildLinearSystem(correspondences, x, kernel_scale);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        x.update(dx);
        // Termination criteria
        if (dx.norm() < convergence_criterion_) break;
    }
    // Spit the final transformation
    return x;
}

}  // namespace kiss_icp
