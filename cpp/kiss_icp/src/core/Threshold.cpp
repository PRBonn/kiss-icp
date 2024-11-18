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
#include "Threshold.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <sophus/se3.hpp>

namespace kiss_icp {
AdaptiveThreshold::AdaptiveThreshold(double initial_threshold,
                                     double min_motion_threshold,
                                     double max_range)
    : min_motion_threshold_(min_motion_threshold),
      max_range_(max_range),
      model_sse_(initial_threshold * initial_threshold),
      num_samples_(1) {}

void AdaptiveThreshold::UpdateModelDeviation(const Sophus::SE3d &current_deviation) {
    const double model_error = [&]() {
        const double theta = Eigen::AngleAxisd(current_deviation.rotationMatrix()).angle();
        const double delta_rot = 2.0 * max_range_ * std::sin(theta / 2.0);
        const double delta_trans = current_deviation.translation().norm();
        return delta_trans + delta_rot;
    }();
    if (model_error > min_motion_threshold_) {
        model_sse_ += model_error * model_error;
        num_samples_++;
    }
}

}  // namespace kiss_icp
