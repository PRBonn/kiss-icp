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
#include <filesystem>
#include <fstream>
#include <iomanip>
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

constexpr bool debug = false;
constexpr std::array<double, 10> translational_perturbations{-0.1, -0.08, -0.06, -0.04, -0.02,
                                                             0.02, 0.04,  0.06,  0.08,  0.1};
constexpr std::array<double, 10> rotational_perturbations{-0.01, -0.008, -0.006, -0.004, -0.002,
                                                          0.002, 0.004,  0.006,  0.008,  0.01};

auto generate_test_perturbations = [](const double min, const double max) {
    std::array<double, 100> perturbations;
    for (int i = 0; i < 100; ++i) {
        perturbations[i] = min + i * (max - min) / 99;  // 100 equally spaced samples
    }
    return perturbations;
};
const std::array<double, 100> test_perturbations_trans = generate_test_perturbations(-0.1, 0.1);
const std::array<double, 100> test_perturbations_rot = generate_test_perturbations(-0.01, 0.01);

constexpr int num_samples = 11;
constexpr int num_unknowns = 3;
Eigen::Matrix<double, num_samples, num_unknowns> GetQuadraticSystem(
    const std::array<double, 10> &perturbations) {
    Eigen::Matrix<double, num_samples, num_unknowns> A;
    std::transform(perturbations.cbegin(), perturbations.cend(), A.rowwise().begin(),
                   [](const auto &perturbation) {
                       return Eigen::Matrix<double, 1, num_unknowns>{
                           1.0, perturbation, 0.5 * perturbation * perturbation};
                   });
    A.row(num_samples - 1) = Eigen::Matrix<double, 1, num_unknowns>{1.0, 0.0, 0.0};
    return A;
}

const auto &A_trans = GetQuadraticSystem(translational_perturbations);
const auto &A_trans_inv = (A_trans.transpose() * A_trans).ldlt();

const auto &A_rot = GetQuadraticSystem(rotational_perturbations);
const auto &A_rot_inv = (A_rot.transpose() * A_rot).ldlt();

inline double square(double x) { return x * x; }

void WriteResidualPlotSVG(const std::filesystem::path &file_path,
                          const std::array<double, num_samples - 1> &perturbations,
                          const Eigen::Matrix<double, num_samples, 1> &residuals,
                          const std::array<double, 100> &test_perturbations,
                          const Eigen::Matrix<double, 100, 1> &estimated_residuals,
                          const std::string &title) {
    constexpr int width = 900;
    constexpr int height = 600;
    constexpr double margin_left = 90.0;
    constexpr double margin_right = 30.0;
    constexpr double margin_top = 60.0;
    constexpr double margin_bottom = 80.0;

    const auto [x_min_it, x_max_it] =
        std::minmax_element(perturbations.begin(), perturbations.end());
    const auto [y_min_it, y_max_it] = std::minmax_element(residuals.begin(), residuals.end());

    double x_min = *x_min_it;
    double x_max = *x_max_it;
    double y_min = *y_min_it;
    double y_max = *y_max_it;

    for (int i = 0; i < num_samples; ++i) {
        y_min = std::min(y_min, (estimated_residuals)(i));
        y_max = std::max(y_max, (estimated_residuals)(i));
    }

    const double plot_width = width - margin_left - margin_right;
    const double plot_height = height - margin_top - margin_bottom;
    const auto scale_x = [&](double x) {
        return margin_left + (x - x_min) * plot_width / (x_max - x_min);
    };
    const auto scale_y = [&](double y) {
        return margin_top + plot_height - (y - y_min) * plot_height / (y_max - y_min);
    };

    std::filesystem::create_directories(file_path.parent_path());
    std::ofstream out(file_path);
    if (!out) {
        std::cerr << "Could not open plot output file: " << file_path << '\n';
        return;
    }

    out << std::fixed << std::setprecision(6);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "  <rect x=\"0\" y=\"0\" width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    out << "  <text x=\"" << width / 2 << "\" y=\"32\" text-anchor=\"middle\""
        << " font-family=\"times\" font-size=\"24\" fill=\"#111\">" << title << "</text>\n";

    // Axes
    const double x_axis_y = scale_y(0.0);
    const double y_axis_x = scale_x(0.0);
    out << "  <line x1=\"" << margin_left << "\" y1=\"" << x_axis_y << "\" x2=\""
        << width - margin_right << "\" y2=\"" << x_axis_y
        << "\" stroke=\"#333\" stroke-width=\"1.5\"/>\n";
    out << "  <line x1=\"" << y_axis_x << "\" y1=\"" << margin_top << "\" x2=\"" << y_axis_x
        << "\" y2=\"" << height - margin_bottom << "\" stroke=\"#333\" stroke-width=\"1.5\"/>\n";

    // Grid and tick labels
    constexpr int ticks = 5;
    for (int i = 0; i <= ticks; ++i) {
        const double t = static_cast<double>(i) / ticks;
        const double x = x_min + t * (x_max - x_min);
        const double sx = scale_x(x);
        out << "  <line x1=\"" << sx << "\" y1=\"" << margin_top << "\" x2=\"" << sx << "\" y2=\""
            << height - margin_bottom << "\" stroke=\"#eee\" stroke-width=\"1\"/>\n";
        out << "  <text x=\"" << sx << "\" y=\"" << height - 40
            << "\" text-anchor=\"middle\" font-family=\"times\" font-size=\"12\" "
               "fill=\"#444\">"
            << x << "</text>\n";

        const double y = y_min + t * (y_max - y_min);
        const double sy = scale_y(y);
        out << "  <line x1=\"" << margin_left << "\" y1=\"" << sy << "\" x2=\""
            << width - margin_right << "\" y2=\"" << sy
            << "\" stroke=\"#eee\" stroke-width=\"1\"/>\n";
        out << "  <text x=\"" << margin_left - 12 << "\" y=\"" << sy + 4
            << "\" text-anchor=\"end\" font-family=\"times\" font-size=\"12\" fill=\"#444\">" << y
            << "</text>\n";
    }

    out << "  <text x=\"" << width / 2 << "\" y=\"" << height - 20
        << "\" text-anchor=\"middle\" font-family=\"times\" font-size=\"16\" fill=\"#111\">"
        << "Perturbation" << "</text>\n";
    out << "  <text x=\"24\" y=\"" << height / 2
        << "\" text-anchor=\"middle\" font-family=\"times\" font-size=\"16\" fill=\"#111\" "
           "transform=\"rotate(-90 24 "
        << height / 2 << ")\">Residual</text>\n";

    // Estimated residuals from the fitted model
    out << "  <polyline fill=\"none\" stroke=\"#ff7f0e\" stroke-width=\"2.5\" "
           "stroke-dasharray=\"8,6\" points=\"";
    for (int i = 0; i < 100; ++i) {
        out << scale_x(test_perturbations[i]) << ',' << scale_y((estimated_residuals)(i)) << ' ';
    }
    out << "\"/>\n";

    // Sample markers
    for (int i = 0; i < num_samples - 1; ++i) {
        out << "  <circle cx=\"" << scale_x(perturbations[i]) << "\" cy=\"" << scale_y(residuals(i))
            << "\" r=\"4.5\" fill=\"#d62728\" stroke=\"white\" stroke-width=\"1\"/>\n";
    }
    out << "  <circle cx=\"" << scale_x(0.0) << "\" cy=\"" << scale_y(residuals(num_samples - 1))
        << "\" r=\"4.5\" fill=\"#27d633\" stroke=\"white\" stroke-width=\"1\"/>\n";

    // Legend
    const double legend_x = width - 255.0;
    const double legend_y = margin_top + 18.0;
    out << "  <rect x=\"" << legend_x << "\" y=\"" << legend_y - 24
        << "\" width=\"220\" height=\"100\" rx=\"8\" fill=\"white\" stroke=\"#ddd\"/>\n";
    out << "  <circle cx=\"" << legend_x + 34 << "\" cy=\"" << legend_y
        << "\" r=\"4.5\" fill=\"#d62728\" stroke=\"white\" stroke-width=\"1\"/>\n";
    out << "  <text x=\"" << legend_x + 64 << "\" y=\"" << legend_y + 4
        << "\" font-family=\"times\" font-size=\"13\" fill=\"#111\">Train "
           "samples</text>\n";
    out << "  <circle cx=\"" << legend_x + 34 << "\" cy=\"" << legend_y + 26
        << "\" r=\"4.5\" fill=\"#27d633\" stroke=\"white\" stroke-width=\"1\"/>\n";
    out << "  <text x=\"" << legend_x + 64 << "\" y=\"" << legend_y + 30
        << "\" font-family=\"times\" font-size=\"13\" fill=\"#111\">Optimized pose</text>\n";
    out << "  <circle cx=\"" << legend_x + 34 << "\" cy=\"" << legend_y + 52
        << "\" r=\"4.5\" fill=\"#ff7f0e\" stroke=\"white\" stroke-width=\"1\"/>\n";
    out << "  <text x=\"" << legend_x + 64 << "\" y=\"" << legend_y + 56
        << "\" font-family=\"times\" font-size=\"13\" fill=\"#111\">Fitted quadratic "
           "model</text>\n";

    out << "</svg>\n";
}

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
    std::transform(frame.cbegin(), frame.cend(), frame_perturbed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Correspondences correspondences =
        DataAssociation(frame_perturbed, voxel_map, max_distance);
    const double optimal_residual =
        std::accumulate(correspondences.cbegin(), correspondences.cend(), 0.0,
                        [](double sum, const auto &correspondence) {
                            const auto &[source, target] = correspondence;
                            return sum + (source - target).squaredNorm();
                        }) /
        static_cast<double>(correspondences.size());
    Eigen::Matrix<double, 6, num_unknowns> parameters =
        Eigen::Matrix<double, 6, num_unknowns>::Zero();

    for (int d = 0; d < 3; ++d) {
        Eigen::Matrix<double, num_samples, 1> residuals;
        std::transform(
            translational_perturbations.cbegin(), translational_perturbations.cend(),
            residuals.begin(), [&](const auto val) {
                Eigen::Vector6d dx = Eigen::Vector6d::Zero();
                dx(d) = val;
                const Sophus::SE3d perturbation(Sophus::SO3d::exp(dx.tail<3>()), dx.head<3>());
                std::transform(frame.cbegin(), frame.cend(), frame_perturbed.begin(),
                               [&](const auto &point) { return pose * perturbation * point; });
                const Correspondences correspondences =
                    DataAssociation(frame_perturbed, voxel_map, max_distance);
                const double residual =
                    std::accumulate(correspondences.cbegin(), correspondences.cend(), 0.0,
                                    [](double sum, const auto &correspondence) {
                                        const auto &[source, target] = correspondence;
                                        return sum + (source - target).squaredNorm();
                                    }) /
                    static_cast<double>(correspondences.size());
                return residual;
            });
        residuals(num_samples - 1) = optimal_residual;
        parameters.row(d) = A_trans_inv.solve(A_trans.transpose() * residuals).transpose();
        if (debug) {
            const std::filesystem::path plot_dir = "results/perturbation_analysis";
            const std::filesystem::path plot_file =
                plot_dir / ("residual_vs_perturbation_dim_" + std::to_string(d) + ".svg");
            Eigen::Matrix<double, 100, 1> estimated_residuals;
            std::transform(test_perturbations_trans.cbegin(), test_perturbations_trans.cend(),
                           estimated_residuals.begin(), [&](const auto dx) {
                               return parameters.row(d).dot(
                                   Eigen::Matrix<double, num_unknowns, 1>{1.0, dx, 0.5 * dx * dx});
                           });
            WriteResidualPlotSVG(plot_file, translational_perturbations, residuals,
                                 test_perturbations_trans, estimated_residuals,
                                 "Residuals vs Perturbation (dimension " + std::to_string(d) + ")");
        }
    }
    for (int d = 3; d < 6; ++d) {
        Eigen::Matrix<double, num_samples, 1> residuals;
        std::transform(
            rotational_perturbations.cbegin(), rotational_perturbations.cend(), residuals.begin(),
            [&](const auto val) {
                Eigen::Vector6d dx = Eigen::Vector6d::Zero();
                dx(d) = val;
                const Sophus::SE3d perturbation(Sophus::SO3d::exp(dx.tail<3>()), dx.head<3>());
                std::transform(frame.cbegin(), frame.cend(), frame_perturbed.begin(),
                               [&](const auto &point) { return pose * perturbation * point; });
                const Correspondences correspondences =
                    DataAssociation(frame_perturbed, voxel_map, max_distance);
                const double residual =
                    std::accumulate(correspondences.cbegin(), correspondences.cend(), 0.0,
                                    [](double sum, const auto &correspondence) {
                                        const auto &[source, target] = correspondence;
                                        return sum + (source - target).squaredNorm();
                                    }) /
                    static_cast<double>(correspondences.size());
                return residual;
            });
        residuals(num_samples - 1) = optimal_residual;
        parameters.row(d) = A_rot_inv.solve(A_rot.transpose() * residuals).transpose();

        if (debug) {
            const std::filesystem::path plot_dir = "results/perturbation_analysis";
            const std::filesystem::path plot_file =
                plot_dir / ("residual_vs_perturbation_dim_" + std::to_string(d) + ".svg");
            Eigen::Matrix<double, 100, 1> estimated_residuals;
            std::transform(test_perturbations_rot.cbegin(), test_perturbations_rot.cend(),
                           estimated_residuals.begin(), [&](const auto dx) {
                               return parameters.row(d).dot(
                                   Eigen::Matrix<double, num_unknowns, 1>{1.0, dx, 0.5 * dx * dx});
                           });

            WriteResidualPlotSVG(plot_file, rotational_perturbations, residuals,
                                 test_perturbations_rot, estimated_residuals,
                                 "Residuals vs Perturbation (dimension " + std::to_string(d) + ")");
        }
    }
    Eigen::Vector6d quadratic_term = parameters.block<6, 1>(0, 2);

    std::transform(
        quadratic_term.begin(), quadratic_term.end(), quadratic_term.begin(),
        [](double x) { return std::clamp(x, 1e-6, std::numeric_limits<double>::max()); });
    if (debug) std::cout << "Quadratic Hessian Matrix:\n" << quadratic_term << std::endl;
    return quadratic_term.asDiagonal();
}
}  // namespace kiss_icp
