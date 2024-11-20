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
#pragma once

#include <sophus/se3.hpp>

namespace kiss_icp {

struct AdaptiveThreshold {
    explicit AdaptiveThreshold(double initial_threshold,
                               double min_motion_threshold,
                               double max_range);

    /// Update the current belief of the deviation from the prediction model
    void UpdateModelDeviation(const Sophus::SE3d &current_deviation);

    /// Returns the KISS-ICP adaptive threshold used in registration
    inline double ComputeThreshold() const { return std::sqrt(model_sse_ / num_samples_); }

    // configurable parameters
    double min_motion_threshold_;
    double max_range_;

    // Local cache for ccomputation
    double model_sse_;
    int num_samples_;
};

}  // namespace kiss_icp
