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
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

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

constexpr int num_samples = 101;  // Adjust this value as needed
Eigen::Matrix<double, num_samples, 6> LatinHypercubeSampling() {
    // Generate samples in the unit cube
    Eigen::Matrix<double, num_samples, 6> unit_samples;
    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < num_samples - 1; ++i) {
            unit_samples(i, j) =
                (i + static_cast<double>(rand()) / RAND_MAX) / (num_samples * 2.5) - 0.2;
        }
        std::shuffle(unit_samples.col(j).begin(), unit_samples.col(j).end(),
                     std::default_random_engine());
    }
    for (int j = 3; j < 6; ++j) {
        for (int i = 0; i < num_samples - 1; ++i) {
            unit_samples(i, j) =
                (i + static_cast<double>(rand()) / RAND_MAX) / (num_samples * 25.0) - 0.02;
        }
        std::shuffle(unit_samples.col(j).begin(), unit_samples.col(j).end(),
                     std::default_random_engine());
    }

    unit_samples.row(num_samples - 1) =
        Eigen::Matrix<double, 1, 6>::Zero();  // Add the zero perturbation

    return unit_samples;
}

constexpr double sq_kernel_scale = 0.01;

Eigen::Array<double, num_samples, num_samples> GetRBFSystem(
    const Eigen::Matrix<double, num_samples, 6> &unit_samples) {
    Eigen::Array<double, num_samples, num_samples> K;
    std::vector<double> dists(num_samples * num_samples, 0.0);
    for (int i = 0; i < num_samples; ++i) {
        K(i, i) = 1e-6;
        for (int j = 0; j < i; ++j) {
            const auto r = (unit_samples.row(i) - unit_samples.row(j)).squaredNorm();
            K(i, j) = r;
            K(j, i) = r;
            dists[i * num_samples + j] = r;
            dists[j * num_samples + i] = r;
        }
    }
    K = (-0.5 * K / sq_kernel_scale).exp();
    return K;
}

const Eigen::Matrix<double, num_samples, 6> &lhs_samples = LatinHypercubeSampling();

double EvaluateRBF(const Eigen::Matrix<double, num_samples, 1> &weights,
                   const Eigen::Matrix<double, 1, 6> &query) {
    Eigen::Matrix<double, num_samples, 1> k_query;
    for (int i = 0; i < num_samples; ++i) {
        k_query(i) = (query - lhs_samples.row(i)).squaredNorm();
    }
    k_query = (-0.5 * k_query / sq_kernel_scale).array().exp();
    return weights.transpose() * k_query;
}

const Eigen::Matrix<double, num_samples, 6> &lhs_eval_samples = LatinHypercubeSampling();

const auto &rbf_system_K = GetRBFSystem(lhs_samples);
const auto &K_inv = rbf_system_K.matrix().ldlt();

inline double square(double x) { return x * x; }

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
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
    const auto hessian = PerturbationAnalysis(frame, voxel_map, T_icp * initial_guess, 5.0);
    // Spit the final transformation
    return {T_icp * initial_guess, hessian};
}

// perturbation analysis
Eigen::Matrix<double, 6, 6> Registration::PerturbationAnalysis(
    const std::vector<Eigen::Vector3d> &frame,
    const VoxelHashMap &voxel_map,
    const Sophus::SE3d &pose,
    const double max_distance) {
    std::vector<Eigen::Vector3d> frame_perturbed(frame.size());
    Eigen::Matrix<double, num_samples, 1> residuals;
    std::transform(
        lhs_samples.rowwise().begin(), lhs_samples.rowwise().end(), residuals.begin(),
        [&](const auto &lhs_sample) {
            const Sophus::SE3d perturbation(
                Sophus::SO3d::exp(Eigen::Vector3d(lhs_sample(3), lhs_sample(4), lhs_sample(5))),
                Eigen::Vector3d(lhs_sample(0), lhs_sample(1), lhs_sample(2)));
            std::transform(frame.cbegin(), frame.cend(), frame_perturbed.begin(),
                           [&](const auto &point) { return pose * perturbation * point; });
            const Correspondences correspondences_perturbed =
                DataAssociation(frame_perturbed, voxel_map, max_distance);
            const double residual_perturb =
                std::accumulate(correspondences_perturbed.cbegin(),
                                correspondences_perturbed.cend(), 0.0,
                                [](double sum, const auto &correspondence) {
                                    const auto &[source, target] = correspondence;
                                    return sum + (source - target).squaredNorm();
                                }) /
                static_cast<double>(correspondences_perturbed.size());
            return residual_perturb;
        });

    double pred_error = 0.0;
    const Eigen::Matrix<double, num_samples, 1> rbf_weights = (K_inv.solve(residuals)).transpose();
    std::for_each(
        lhs_eval_samples.rowwise().begin(), lhs_eval_samples.rowwise().end(),
        [&](const auto &lhs_sample) {
            const Sophus::SE3d perturbation(
                Sophus::SO3d::exp(Eigen::Vector3d(lhs_sample(3), lhs_sample(4), lhs_sample(5))),
                Eigen::Vector3d(lhs_sample(0), lhs_sample(1), lhs_sample(2)));
            std::transform(frame.cbegin(), frame.cend(), frame_perturbed.begin(),
                           [&](const auto &point) { return pose * perturbation * point; });
            const Correspondences correspondences_perturbed =
                DataAssociation(frame_perturbed, voxel_map, max_distance);
            const double residual_perturb =
                std::accumulate(correspondences_perturbed.cbegin(),
                                correspondences_perturbed.cend(), 0.0,
                                [](double sum, const auto &correspondence) {
                                    const auto &[source, target] = correspondence;
                                    return sum + (source - target).squaredNorm();
                                }) /
                static_cast<double>(correspondences_perturbed.size());
            pred_error += std::abs(EvaluateRBF(rbf_weights, lhs_sample) - residual_perturb);
        });
    std::cout << "Average prediction error at LHS eval samples: " << pred_error / num_samples
              << std::endl;
    // Evaluate RBF hessian at the zero perturbation
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < num_samples; ++i) {
        hessian += rbf_weights(i) * rbf_system_K(i, num_samples - 1) *
                   (((lhs_samples.row(i).transpose() * lhs_samples.row(i)) / sq_kernel_scale) -
                    Eigen::Matrix<double, 6, 6>::Identity());
    }
    hessian /= sq_kernel_scale;
    // Make Symmetric for Information gain Matrix
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(hessian);
    auto eigvalues = eigen_solver.eigenvalues();
    auto eigvectors = eigen_solver.eigenvectors();
    std::cout << "Raw Perturbation Analysis Hessian:\n" << hessian << std::endl;
    std::cout << "Eigenvalues:\n" << eigvalues.transpose() << std::endl;
    // Clamp eigenvalues to avoid numerical issues in the information gain computation
    std::transform(eigvalues.begin(), eigvalues.end(), eigvalues.begin(), [](double x) {
        return std::clamp(x, 1e-6, std::numeric_limits<double>::max());
    });

    // Compute the information gain as the log of the determinant of the hessian
    std::clamp(eigvalues(0), 1e-6, std::numeric_limits<double>::max());
    eigvalues = eigvalues / eigvalues.maxCoeff();  // Normalize eigenvalues for better visualization
    const Eigen::Matrix<double, 6, 6> hessian_positive_definite =
        eigvectors * eigvalues.asDiagonal() * eigvectors.transpose();
    // std::cout << "Perturbation Analysis Hessian:\n" << hessian_positive_definite << std::endl;
    return hessian_positive_definite;
}
}  // namespace kiss_icp
