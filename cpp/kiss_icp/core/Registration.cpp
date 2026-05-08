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
#include <tbb/blocked_range2d.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <execution>
#include <iostream>
#include <numeric>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>
#include <vector>

#include "VoxelHashMap.hpp"
#include "VoxelUtils.hpp"

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

namespace {
auto duration_ms = [](auto start, auto end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
};
constexpr int num_samples = 11;
constexpr int num_unknowns = 3;
constexpr int dof = 6;

constexpr std::array<double, num_samples - 1> trans_perturb{-0.1, -0.08, -0.06, -0.04, -0.02,
                                                            0.02, 0.04,  0.06,  0.08,  0.1};
constexpr std::array<double, num_samples - 1> rot_perturb{-0.01, -0.008, -0.006, -0.004, -0.002,
                                                          0.002, 0.004,  0.006,  0.008,  0.01};

auto get_se3_perturbations = []() {
    std::array<std::array<Sophus::SE3d, num_samples - 1>, dof> perturbations;
    for (int i = 0; i < num_samples - 1; ++i) {
        perturbations[0][i] =
            Sophus::SE3d(Eigen::Matrix3d::Identity(), trans_perturb[i] * Eigen::Vector3d::UnitX());

        perturbations[1][i] =
            Sophus::SE3d(Eigen::Matrix3d::Identity(), trans_perturb[i] * Eigen::Vector3d::UnitY());

        perturbations[2][i] =
            Sophus::SE3d(Eigen::Matrix3d::Identity(), trans_perturb[i] * Eigen::Vector3d::UnitZ());

        perturbations[3][i] =
            Sophus::SE3d(Sophus::SO3d::rotX(rot_perturb[i]), Eigen::Vector3d::Zero());

        perturbations[4][i] =
            Sophus::SE3d(Sophus::SO3d::rotY(rot_perturb[i]), Eigen::Vector3d::Zero());

        perturbations[5][i] =
            Sophus::SE3d(Sophus::SO3d::rotZ(rot_perturb[i]), Eigen::Vector3d::Zero());
    }
    return perturbations;
};

const auto se3_perturbations = get_se3_perturbations();

Eigen::Matrix<double, num_samples, num_unknowns> GetQuadraticSystem(
    const std::array<double, num_samples - 1> &perturbations) {
    Eigen::Matrix<double, num_samples, num_unknowns> A;
    std::transform(perturbations.cbegin(), perturbations.cend(), A.rowwise().begin(),
                   [](const auto &perturbation) {
                       return Eigen::Matrix<double, 1, num_unknowns>{
                           1.0, perturbation, 0.5 * perturbation * perturbation};
                   });
    A.row(num_samples - 1) = Eigen::Matrix<double, 1, num_unknowns>{1.0, 0.0, 0.0};
    return A;
}

const auto A_trans = GetQuadraticSystem(trans_perturb);
const auto N_trans = (A_trans.transpose() * A_trans).inverse() * A_trans.transpose();

const auto A_rot = GetQuadraticSystem(rot_perturb);
const auto N_rot = (A_rot.transpose() * A_rot).inverse() * A_rot.transpose();

const std::array<Eigen::Matrix<double, num_unknowns, num_samples>, dof> N{N_trans, N_trans, N_trans,
                                                                          N_rot,   N_rot,   N_rot};

inline double square(double x) { return x * x; }

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

double PointToPointResidual(const std::vector<Eigen::Vector3d> &points,
                            const kiss_icp::VoxelHashMap &voxel_map,
                            const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    std::vector<double> residuals;
    residuals.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
            std::for_each(r.begin(), r.end(), [&](const auto &point) {
                const auto &[_, distance] = voxel_map.GetClosestNeighbor(point);
                if (distance < max_correspondance_distance) {
                    residuals.emplace_back(distance);
                }
            });
        });

    const double mean_residual = std::reduce(residuals.cbegin(), residuals.cend(), 0.0) /
                                 static_cast<double>(residuals.size());

    return mean_residual;
}

Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const kiss_icp::VoxelHashMap &voxel_map,
                                const double max_correspondance_distance) {
    using points_iterator = std::vector<Eigen::Vector3d>::const_iterator;
    Correspondences correspondences;
    correspondences.reserve(points.size());
    tbb::parallel_for(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
            std::for_each(r.begin(), r.end(), [&](const auto &point) {
                const auto &[closest_neighbor, distance] = voxel_map.GetClosestNeighbor(point);
                if (distance < max_correspondance_distance) {
                    correspondences.emplace_back(point, closest_neighbor);
                }
            });
        });
    return correspondences;
}

LinearSystem BuildLinearSystem(const Correspondences &correspondences, const double kernel_scale) {
    auto compute_jacobian_and_residual = [](const auto &correspondence) {
        const auto &[source, target] = correspondence;
        const Eigen::Vector3d residual = source - target;
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
        return std::make_tuple(J_r, residual);
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

Registration::Registration(int max_num_iteration, double convergence_criterion, int max_num_threads)
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

std::pair<Sophus::SE3d, Eigen::Matrix<double, 6, 6>> Registration::AlignPointsToMap(
    const std::vector<Eigen::Vector3d> &frame,
    const VoxelHashMap &voxel_map,
    const Sophus::SE3d &initial_guess,
    const double max_distance,
    const double kernel_scale) {
    if (voxel_map.Empty()) return {initial_guess, Eigen::Matrix<double, 6, 6>::Identity()};

    // Equation (9)
    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        // Equation (10)
        const auto correspondences = DataAssociation(source, voxel_map, max_distance);
        // Equation (11)
        const auto &[JTJ, JTr] = BuildLinearSystem(correspondences, kernel_scale);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        // Equation (12)
        TransformPoints(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        if (dx.norm() < convergence_criterion_) break;
    }
    const auto hessian =
        PerturbationAnalysis(frame, voxel_map, T_icp * initial_guess, max_distance);
    // Spit the final transformation
    return {T_icp * initial_guess, hessian};
}

// perturbation analysis
Eigen::Matrix<double, 6, 6> Registration::PerturbationAnalysis(
    const std::vector<Eigen::Vector3d> &frame,
    const VoxelHashMap &voxel_map,
    const Sophus::SE3d &pose,
    const double max_distance) {
    auto t_start = std::chrono::high_resolution_clock::now();

    std::vector<Eigen::Vector3d> frame_perturbed(frame.size());
    std::transform(frame.cbegin(), frame.cend(), frame_perturbed.begin(),
                   [&](const auto &point) { return pose * point; });

    const double optimal_residual = PointToPointResidual(frame_perturbed, voxel_map, max_distance);
    Eigen::Matrix<double, dof, num_unknowns> parameters =
        Eigen::Matrix<double, dof, num_unknowns>::Zero();

    auto t_perturb_start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix<double, num_samples, dof> residuals;
    residuals.row(num_samples - 1) = Eigen::Matrix<double, 1, dof>::Constant(optimal_residual);
    tbb::parallel_for(tbb::blocked_range<int>{0, dof}, [&](const tbb::blocked_range<int> &r1) {
        for (int d = r1.begin(); d < r1.end(); ++d) {
            const auto &perturbations = se3_perturbations[d];
            tbb::parallel_for(
                tbb::blocked_range<int>{0, num_samples - 1},
                [&](const tbb::blocked_range<int> &r2) {
                    for (int i = r2.begin(); i < r2.end(); ++i) {
                        const Sophus::SE3d perturbed_pose = pose * perturbations[i];
                        std::vector<Eigen::Vector3d> frame_perturbed_local(frame.size());
                        std::transform(frame.cbegin(), frame.cend(), frame_perturbed_local.begin(),
                                       [&](const auto &point) { return perturbed_pose * point; });
                        const double residual =
                            PointToPointResidual(frame_perturbed_local, voxel_map, max_distance);
                        residuals(i, d) = residual;
                    }
                });
        }
    });

    std::transform(
        N.cebgin(), N.cend(), residuals.colwise().cbegin(), parameters.rowwise().begin(),
        [&](const auto &N_d, const auto &residuals_d) { return (N_d * residuals_d).transpose(); });
    auto t_perturb_end = std::chrono::high_resolution_clock::now();

    Eigen::Vector6d quadratic_term = parameters.block<dof, 1>(0, 2);

    std::transform(
        quadratic_term.begin(), quadratic_term.end(), quadratic_term.begin(),
        [](double x) { return std::clamp(x, 1e-6, std::numeric_limits<double>::max()); });

    auto t_end = std::chrono::high_resolution_clock::now();

    // Print timing breakdown

    std::cout << "\n=== PerturbationAnalysis Timing Breakdown ===\n";
    std::cout << "Perturbation loop:    " << duration_ms(t_perturb_start, t_perturb_end) << " ms\n";
    std::cout << "Total:                " << duration_ms(t_start, t_end) << " ms\n";
    std::cout << "=========================================\n\n";

    return quadratic_term.asDiagonal();
}
}  // namespace kiss_icp
