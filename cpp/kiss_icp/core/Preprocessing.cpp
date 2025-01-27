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

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <functional>
#include <vector>

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
                                                      const std::vector<double> &timestamps,
                                                      const Sophus::SE3d &relative_motion) const {
    const std::vector<Eigen::Vector3d> &deskewed_frame = [&]() {
        if (!deskew_ || timestamps.empty()) {
            return frame;
        } else {
            const auto &[min, max] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
            const double min_time = *min;
            const double max_time = *max;
            const auto normalize = [&](const double t) {
                return (t - min_time) / (max_time - min_time);
            };
            const auto &omega = relative_motion.log();
            std::vector<Eigen::Vector3d> deskewed_frame(frame.size());
            tbb::parallel_for(
                // Index Range
                tbb::blocked_range<size_t>{0, deskewed_frame.size()},
                // Parallel Compute
                [&](const tbb::blocked_range<size_t> &r) {
                    for (size_t idx = r.begin(); idx < r.end(); ++idx) {
                        const auto &point = frame.at(idx);
                        const auto &stamp = normalize(timestamps.at(idx));
                        const auto pose = Sophus::SE3d::exp((stamp - 1.0) * omega);
                        deskewed_frame.at(idx) = pose * point;
                    };
                });
            return deskewed_frame;
        }
    }();
    std::vector<Eigen::Vector3d> preprocessed_frame;
    preprocessed_frame.reserve(deskewed_frame.size());
    std::for_each(deskewed_frame.cbegin(), deskewed_frame.cend(), [&](const auto &point) {
        const double point_range = point.norm();
        if (point_range < max_range_ && point_range > min_range_) {
            preprocessed_frame.emplace_back(point);
        }
    });
    preprocessed_frame.shrink_to_fit();
    return preprocessed_frame;
}
}  // namespace kiss_icp
