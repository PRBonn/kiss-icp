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

namespace {
inline double square(double x) { return x * x; }

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}

double PointToPointResidual(const std::vector<Eigen::Vector3d> &points,
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
    for (auto &local : local_residuals) {
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

constexpr int num_samples = 6;

constexpr std::array<double, num_samples> trans_perturb{-0.1, -0.06, -0.02, 0.02, 0.06, 0.1};
constexpr std::array<double, num_samples> rot_perturb{-0.01, -0.006, -0.002, 0.002, 0.006, 0.01};

auto get_se3_perturbations_trans = []() {
    std::array<std::array<Sophus::SE3d, 6>, num_samples> perturbations;
    for (int i = 0; i < num_samples; ++i) {
        perturbations[i][0] =
            Sophus::SE3d(Eigen::Matrix3d::Identity(), trans_perturb[i] * Eigen::Vector3d::UnitX());

        perturbations[i][1] =
            Sophus::SE3d(Eigen::Matrix3d::Identity(), trans_perturb[i] * Eigen::Vector3d::UnitY());

        perturbations[i][2] =
            Sophus::SE3d(Eigen::Matrix3d::Identity(), trans_perturb[i] * Eigen::Vector3d::UnitZ());

        perturbations[i][3] = Sophus::SE3d(
            Eigen::Matrix3d::Identity(), (1.0 / std::sqrt(2)) * trans_perturb[i] *
                                             (Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitY()));

        perturbations[i][4] = Sophus::SE3d(
            Eigen::Matrix3d::Identity(), (1.0 / std::sqrt(2)) * trans_perturb[i] *
                                             (Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitZ()));

        perturbations[i][5] = Sophus::SE3d(
            Eigen::Matrix3d::Identity(), (1.0 / std::sqrt(2)) * trans_perturb[i] *
                                             (Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitZ()));
    }
    return perturbations;
};

auto get_se3_perturbations_rot = []() {
    std::array<std::array<Sophus::SE3d, 6>, num_samples> perturbations;
    for (int i = 0; i < num_samples; ++i) {
        perturbations[i][0] =
            Sophus::SE3d(Sophus::SO3d::rotX(rot_perturb[i]), Eigen::Vector3d::Zero());

        perturbations[i][1] =
            Sophus::SE3d(Sophus::SO3d::rotY(rot_perturb[i]), Eigen::Vector3d::Zero());

        perturbations[i][2] =
            Sophus::SE3d(Sophus::SO3d::rotZ(rot_perturb[i]), Eigen::Vector3d::Zero());

        perturbations[i][3] =
            Sophus::SE3d(Sophus::SO3d::exp((1.0 / std::sqrt(2)) * rot_perturb[i] *
                                           (Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitY())),
                         Eigen::Vector3d::Zero());

        perturbations[i][4] =
            Sophus::SE3d(Sophus::SO3d::exp((1.0 / std::sqrt(2)) * rot_perturb[i] *
                                           (Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitZ())),
                         Eigen::Vector3d::Zero());

        perturbations[i][5] =
            Sophus::SE3d(Sophus::SO3d::exp((1.0 / std::sqrt(2)) * rot_perturb[i] *
                                           (Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitZ())),
                         Eigen::Vector3d::Zero());
    }
    return perturbations;
};

auto get_square_symm_3x3_from_vec6d = [](const Eigen::Vector6d &vec) {
    Eigen::Matrix3d mat;
    mat << vec(0), vec(3), vec(5), vec(3), vec(1), vec(4), vec(5), vec(4), vec(2);
    return mat;
};

const auto se3_perturbations_trans = get_se3_perturbations_trans();
const auto se3_perturbations_rot = get_se3_perturbations_rot();

Eigen::Matrix<double, 9, num_samples * 6> GetQuadraticSystem(
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

const Eigen::Matrix<double, 9, num_samples * 6> N_trans = GetQuadraticSystem(trans_perturb);
const Eigen::Matrix<double, 9, num_samples * 6> N_rot = GetQuadraticSystem(rot_perturb);
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
    const double optimal_residual =
        PointToPointResidual(frame_downsampled, pose, voxel_map, max_correspondence_distance);

    Eigen::Matrix<double, num_samples * 6, 1> residuals_trans;
    tbb::parallel_for(0, 6 * num_samples, [&](const int idx) {
        const int dim = idx / num_samples;
        const int n = idx % num_samples;
        const Sophus::SE3d perturbed_pose = pose * se3_perturbations_trans[dim][n];
        residuals_trans(idx) = PointToPointResidual(frame_downsampled, perturbed_pose, voxel_map,
                                                    max_correspondence_distance) -
                               optimal_residual;
    });

    Eigen::Matrix<double, num_samples * 6, 1> residuals_rot;
    tbb::parallel_for(0, 6 * num_samples, [&](const int idx) {
        const int dim = idx / num_samples;
        const int n = idx % num_samples;
        const Sophus::SE3d perturbed_pose = pose * se3_perturbations_rot[dim][n];
        residuals_rot(idx) = PointToPointResidual(frame_downsampled, perturbed_pose, voxel_map,
                                                  max_correspondence_distance) -
                             optimal_residual;
    });

    const Eigen::Matrix3d hessian_trans =
        get_square_symm_3x3_from_vec6d((N_trans * residuals_trans).tail<6>());
    const Eigen::Matrix3d hessian_rot =
        get_square_symm_3x3_from_vec6d((N_rot * residuals_rot).tail<6>());

    Eigen::Matrix6d hessian = Eigen::Matrix6d::Zero();

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_trans(hessian_trans);
    auto eigenvalues_trans = solver_trans.eigenvalues();
    const auto eigenvectors_trans = solver_trans.eigenvectors();
    for (int i = 0; i < 3; ++i) {
        if (eigenvalues_trans(i) < 1e-3) {
            std::cout << "trans eigenvalue too small: " << eigenvectors_trans(i) << std::endl;
            eigenvalues_trans(i) = 1e-3;
        }
    }
    hessian.block<3, 3>(0, 0) =
        eigenvectors_trans * eigenvalues_trans.asDiagonal() * eigenvectors_trans.transpose();

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_rot(hessian_rot);
    auto eigenvalues_rot = solver_rot.eigenvalues();
    const auto eigenvectors_rot = solver_rot.eigenvectors();
    for (int i = 0; i < 3; ++i) {
        if (eigenvalues_rot(i) < 1e-3) {
            std::cout << "rot eigenvalue too small: " << eigenvectors_rot(i) << std::endl;
            eigenvalues_rot(i) = 1e-3;
        }
    }
    hessian.block<3, 3>(3, 3) =
        eigenvectors_rot * eigenvalues_rot.asDiagonal() * eigenvectors_rot.transpose();

    return hessian;
}
}  // namespace kiss_icp
