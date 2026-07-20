// MIT License
//
// Copyright (c) 2026 Saurabh Gupta, Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
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
#include <tbb/enumerable_thread_specific.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
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
using RowVector9d = Eigen::Matrix<double, 1, 9>;
}  // namespace Eigen

using Correspondences = std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

constexpr int num_samples = 6;

constexpr std::array<double, num_samples> lin_perturb_val{-0.1, -0.06, -0.02, 0.02, 0.06, 0.1};
constexpr std::array<double, num_samples> ang_perturb_val{-0.01, -0.006, -0.002,
                                                          0.002, 0.006,  0.01};
using LinearSystemUncertainty = Eigen::Matrix<double, 9, num_samples * 6>;

namespace {
inline double square(double x) { return x * x; }

auto to_square_symettric = [](const Eigen::Vector6d &vec) {
    Eigen::Matrix3d mat;
    mat << vec(0), vec(3), vec(5), vec(3), vec(1), vec(4), vec(5), vec(4), vec(2);
    return mat;
};

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

double GetIcpResidual(const std::vector<Eigen::Vector3d> &points,
                      const Sophus::SE3d &pose,
                      const kiss_icp::VoxelHashMap &voxel_map,
                      const double max_correspondance_distance) {
    tbb::enumerable_thread_specific<std::vector<double>> local_residuals;
    tbb::parallel_for(size_t{0}, points.size(), [&](size_t i) {
        const auto &[_, distance] = voxel_map.GetClosestNeighbor(pose * points[i]);
        if (distance < max_correspondance_distance) {
            local_residuals.local().emplace_back(distance);
        }
    });

    double sum_residual = 0.0;
    size_t num_residuals = 0;
    for (const auto &local : local_residuals) {
        sum_residual += std::reduce(local.cbegin(), local.cend(), 0.0);
        num_residuals += local.size();
    }

    const double mean_residual = num_residuals > 0
                                     ? sum_residual / static_cast<double>(num_residuals)
                                     : max_correspondance_distance;
    return mean_residual;
}

Correspondences DataAssociation(const std::vector<Eigen::Vector3d> &points,
                                const kiss_icp::VoxelHashMap &voxel_map,
                                const double max_correspondance_distance) {
    tbb::enumerable_thread_specific<Correspondences> local_correspondences;
    tbb::parallel_for(size_t{0}, points.size(), [&](size_t i) {
        const auto &[closest_neighbor, distance] = voxel_map.GetClosestNeighbor(points[i]);
        if (distance < max_correspondance_distance) {
            local_correspondences.local().emplace_back(points[i], closest_neighbor);
        }
    });

    Correspondences correspondences;
    correspondences.reserve(points.size());
    for (auto &local : local_correspondences) {
        correspondences.insert(correspondences.end(), std::make_move_iterator(local.begin()),
                               std::make_move_iterator(local.end()));
    }
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
    auto [JTJ, JTr] = tbb::parallel_reduce(
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

    return {std::move(JTJ), std::move(JTr)};
}

auto get_lin_perturbations_se3 = []() {
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d ux = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d uy = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d uz = Eigen::Vector3d::UnitZ();
    std::array<std::array<Sophus::SE3d, 6>, num_samples> samples;
    for (int i = 0; i < num_samples; ++i) {
        samples[i][0] = Sophus::SE3d(I, lin_perturb_val[i] * ux);
        samples[i][1] = Sophus::SE3d(I, lin_perturb_val[i] * uy);
        samples[i][2] = Sophus::SE3d(I, lin_perturb_val[i] * uz);
        samples[i][3] = Sophus::SE3d(I, (1.0 / std::sqrt(2)) * lin_perturb_val[i] * (ux + uy));
        samples[i][4] = Sophus::SE3d(I, (1.0 / std::sqrt(2)) * lin_perturb_val[i] * (uy + uz));
        samples[i][5] = Sophus::SE3d(I, (1.0 / std::sqrt(2)) * lin_perturb_val[i] * (ux + uz));
    }
    return samples;
};

auto get_ang_perturbations_se3 = []() {
    const Eigen::Vector3d zeros = Eigen::Vector3d::Zero();
    const Eigen::Vector3d ux = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d uy = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d uz = Eigen::Vector3d::UnitZ();
    std::array<std::array<Sophus::SE3d, 6>, num_samples> samples;
    for (int i = 0; i < num_samples; ++i) {
        samples[i][0] = Sophus::SE3d(Sophus::SO3d::rotX(ang_perturb_val[i]), zeros);
        samples[i][1] = Sophus::SE3d(Sophus::SO3d::rotY(ang_perturb_val[i]), zeros);
        samples[i][2] = Sophus::SE3d(Sophus::SO3d::rotZ(ang_perturb_val[i]), zeros);
        samples[i][3] = Sophus::SE3d(
            Sophus::SO3d::exp((1.0 / std::sqrt(2)) * ang_perturb_val[i] * (ux + uy)), zeros);
        samples[i][4] = Sophus::SE3d(
            Sophus::SO3d::exp((1.0 / std::sqrt(2)) * ang_perturb_val[i] * (uy + uz)), zeros);
        samples[i][5] = Sophus::SE3d(
            Sophus::SO3d::exp((1.0 / std::sqrt(2)) * ang_perturb_val[i] * (ux + uz)), zeros);
    }
    return samples;
};

const auto lin_perturbations_se3 = get_lin_perturbations_se3();
const auto ang_perturbations_se3 = get_ang_perturbations_se3();

LinearSystemUncertainty BuildLinearSystemUncertainty(
    const std::array<double, num_samples> &perturbations) {
    Eigen::Matrix<double, num_samples * 6, 9> A;
    for (int n = 0; n < num_samples; ++n) {
        const double x = perturbations[n];
        const double y = perturbations[n];
        const double z = perturbations[n];
        const double xy = perturbations[n] / std::sqrt(2.0);
        const double yz = perturbations[n] / std::sqrt(2.0);
        const double xz = perturbations[n] / std::sqrt(2.0);

        A.row(n * num_samples) =
            Eigen::RowVector9d{x, 0.0, 0.0, 0.5 * x * x, 0.0, 0.0, 0.0, 0.0, 0.0};
        A.row(n * num_samples + 1) =
            Eigen::RowVector9d{0.0, y, 0.0, 0.0, 0.5 * y * y, 0.0, 0.0, 0.0, 0.0};
        A.row(n * num_samples + 2) =
            Eigen::RowVector9d{0.0, 0.0, z, 0.0, 0.0, 0.5 * z * z, 0.0, 0.0, 0.0};
        A.row(n * num_samples + 3) =
            Eigen::RowVector9d{xy, xy, 0.0, 0.5 * xy * xy, 0.5 * xy * xy, 0.0, xy * xy, 0.0, 0.0};
        A.row(n * num_samples + 4) =
            Eigen::RowVector9d{0.0, yz, yz, 0.0, 0.5 * yz * yz, 0.5 * yz * yz, 0.0, yz * yz, 0.0};
        A.row(n * num_samples + 5) =
            Eigen::RowVector9d{xz, 0.0, xz, 0.5 * xz * xz, 0.0, 0.5 * xz * xz, 0.0, 0.0, xz * xz};
    }
    return ((A.transpose() * A).inverse() * A.transpose());
}

const LinearSystemUncertainty N_lin = BuildLinearSystemUncertainty(lin_perturb_val);
const LinearSystemUncertainty N_ang = BuildLinearSystemUncertainty(ang_perturb_val);
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

Sophus::SE3d Registration::AlignPointsToMap(const std::vector<Eigen::Vector3d> &frame,
                                            const VoxelHashMap &voxel_map,
                                            const Sophus::SE3d &initial_guess,
                                            const double max_distance,
                                            const double kernel_scale) {
    if (voxel_map.Empty()) return initial_guess;

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
    const auto correspondences = DataAssociation(source, voxel_map, max_distance);
    fitness_ = static_cast<double>(correspondences.size()) / static_cast<double>(voxel_map.size());
    // Spit the final transformation
    return T_icp * initial_guess;
}

// perturbation analysis
Eigen::Matrix6d Registration::GetHessian(const std::vector<Eigen::Vector3d> &frame,
                                         const VoxelHashMap &voxel_map,
                                         const Sophus::SE3d &pose,
                                         const double max_correspondence_distance) {
    const std::vector<Eigen::Vector3d> frame_downsampled =
        VoxelDownsample(frame, voxel_map.voxel_size_ * 3.0);
    const double ref_residual =
        GetIcpResidual(frame_downsampled, pose, voxel_map, max_correspondence_distance);

    Eigen::Matrix<double, num_samples * 6, 1> residuals_lin;
    tbb::parallel_for(0, 6 * num_samples, [&](const int idx) {
        const int dim = idx / num_samples;
        const int n = idx % num_samples;
        const Sophus::SE3d new_pose = pose * lin_perturbations_se3[dim][n];
        residuals_lin(idx) =
            GetIcpResidual(frame_downsampled, new_pose, voxel_map, max_correspondence_distance) -
            ref_residual;
    });

    Eigen::Matrix<double, num_samples * 6, 1> residuals_ang;
    tbb::parallel_for(0, 6 * num_samples, [&](const int idx) {
        const int dim = idx / num_samples;
        const int n = idx % num_samples;
        const Sophus::SE3d new_pose = pose * ang_perturbations_se3[dim][n];
        residuals_ang(idx) =
            GetIcpResidual(frame_downsampled, new_pose, voxel_map, max_correspondence_distance) -
            ref_residual;
    });

    const Eigen::Matrix3d hessian_lin = to_square_symettric((N_lin * residuals_lin).tail<6>());
    const Eigen::Matrix3d hessian_ang = to_square_symettric((N_ang * residuals_ang).tail<6>());

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_lin(hessian_lin);
    auto eigvals_lin = solver_lin.eigenvalues();
    const auto eigvecs_lin = solver_lin.eigenvectors();
    for (int i = 0; i < 3; ++i) {
        if (eigvals_lin(i) < 1e-3) {
            eigvals_lin(i) = 1e-3;
        }
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_ang(hessian_ang);
    auto eigvals_ang = solver_ang.eigenvalues();
    const auto eigvecs_ang = solver_ang.eigenvectors();
    for (int i = 0; i < 3; ++i) {
        if (eigvals_ang(i) < 1e-3) {
            eigvals_ang(i) = 1e-3;
        }
    }

    Eigen::Matrix6d hessian = Eigen::Matrix6d::Zero();
    hessian.block<3, 3>(0, 0) = eigvecs_lin * eigvals_lin.asDiagonal() * eigvecs_lin.transpose();
    hessian.block<3, 3>(3, 3) = eigvecs_ang * eigvals_ang.asDiagonal() * eigvecs_ang.transpose();

    return hessian;
}
}  // namespace kiss_icp
