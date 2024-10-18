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
#include "Preprocessing.hpp"

#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/parallel_reduce.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstddef>
#include <vector>

namespace {
constexpr double mid_pose_timestamp{0.5};

Eigen::Vector3d DeSkewPoint(const Eigen::Vector3d &point,
                            const double timestamp,
                            const Sophus::SE3d &delta) {
    const auto delta_pose = delta.log();
    const auto motion = Sophus::SE3d::exp((timestamp - mid_pose_timestamp) * delta_pose);
    return motion * point;
}

}  // namespace

namespace kiss_icp {

Preprocessor::Preprocessor(const double max_range,
                           const double min_range,
                           const bool deskew,
                           const int max_num_threads)
    : max_range_(max_range),
      min_range_(min_range),
      deskew_(deskew),
      max_num_threads_(max_num_threads > 0 ? max_num_threads
                                           : tbb::this_task_arena::max_concurrency()) {
    // This global variable requires static duration storage to be able to manipulate the max
    // concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
        tbb::global_control::max_allowed_parallelism, static_cast<size_t>(max_num_threads_));
}

std::vector<Eigen::Vector3d> Preprocessor::Preprocess(const std::vector<Eigen::Vector3d> &frame,
                                                      const std::vector<double> &timestamps) const {
    std::vector<Eigen::Vector3d> preprocessed_frame;
    preprocessed_frame.reserve(frame.size());
    preprocessed_frame = tbb::parallel_reduce(
        // Index Range
        tbb::blocked_range<size_t>(0, frame.size()),
        // Initial value
        preprocessed_frame,
        // Parallel Compute
        [&](const tbb::blocked_range<size_t> &r,
            std::vector<Eigen::Vector3d> preprocessed_block) -> std::vector<Eigen::Vector3d> {
            preprocessed_block.reserve(r.size());
            std::for_each(r.begin(), r.end(), [&](const size_t &idx) {
                const auto &point = deskew_ ? DeskewPoint(frame.at(i), timestamps.at(i),
                                                          relative_motion_for_deskewing_)
                                            : frame.at(i);
                const double point_range = point.norm();
                if (point_range < max_range_ && point_range > min_range_) {
                    preprocessed_block.emplace_back(point);
                }
            })
        },
        // Reduction
        [](std::vector<Eigen::Vector3d> a,
           const std::vector<Eigen::Vector3d> &b) -> std::vector<Eigen::Vector3d> {
            a.insert(a.end(),                              //
                     std::make_move_iterator(b.cbegin()),  //
                     std::make_move_iterator(b.cend()));
            return a;
        });
}  // namespace kiss_icp
