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

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <tuple>

namespace {

inline double square(double x) { return x * x; }

inline Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d skew;
    // clang-format off
    skew << 0,      -vec.z(),  vec.y(),
            vec.z(), 0,       -vec.x(),
           -vec.y(), vec.x(),  0;
    // clang-format on
    return skew;
}

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};

Eigen::Matrix4d TransformVector6dToMatrix4d(const Eigen::Vector6d &x) {
    Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
    output.block<3, 3>(0, 0) = (Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(x(1), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(x(0), Eigen::Vector3d::UnitX()))
                                   .matrix();
    output.block<3, 1>(0, 3) = x.block<3, 1>(3, 0);
    return output;
}

}  // namespace

namespace kiss_icp {

std::tuple<Eigen::Vector6d, Eigen::Matrix4d> AlignClouds(const std::vector<Eigen::Vector3d> &source,
                                                         const std::vector<Eigen::Vector3d> &target,
                                                         double th) {
    Eigen::Vector6d x = ComputeUpdate(source, target, th);
    Eigen::Matrix4d update = TransformVector6dToMatrix4d(x);
    return {x, update};
}

Eigen::Vector6d ComputeUpdate(const std::vector<Eigen::Vector3d> &source,
                              const std::vector<Eigen::Vector3d> &target,
                              double th) {
    auto compute_jacobian_and_residual = [&](auto i) {
        const Eigen::Vector3d &source_pt = source[i];
        const Eigen::Vector3d &target_pt = target[i];
        const Eigen::Vector3d residual = (source_pt - target_pt);
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = -1.0 * SkewMatrix(source_pt);
        J_r.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        return std::make_tuple(J_r, residual);
    };

    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<size_t>{0, source.size()},
        // Identity
        ResultTuple(),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto Weight = [&](double residual2) { return square(th) / square(th + residual2); };
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, r] = compute_jacobian_and_residual(i);
                const double w = Weight(r.squaredNorm());
                JTJ_private.noalias() += J_r.transpose() * w * J_r;
                JTr_private.noalias() += J_r.transpose() * w * r;
            }
            return J;
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; });

    return JTJ.ldlt().solve(-JTr);
}

}  // namespace kiss_icp
