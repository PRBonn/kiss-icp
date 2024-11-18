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
#include "Metrics.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <tuple>
#include <vector>

// All this not so beatifull C++ functions are taken from kitti-dev-kit
namespace {
double lengths[] = {100, 200, 300, 400, 500, 600, 700, 800};
int32_t num_lengths = 8;

struct errors {
    int32_t first_frame;
    double r_err;
    double t_err;
    double len;
    double speed;
    errors(int32_t first_frame, double r_err, double t_err, double len, double speed)
        : first_frame(first_frame), r_err(r_err), t_err(t_err), len(len), speed(speed) {}
};

std::vector<double> TrajectoryDistances(const std::vector<Eigen::Matrix4d> &poses) {
    std::vector<double> dist;
    dist.push_back(0);
    for (uint32_t i = 1; i < poses.size(); i++) {
        const Eigen::Matrix4d &P1 = poses[i - 1];
        const Eigen::Matrix4d &P2 = poses[i];

        double dx = P1(0, 3) - P2(0, 3);
        double dy = P1(1, 3) - P2(1, 3);
        double dz = P1(2, 3) - P2(2, 3);

        dist.push_back(dist[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz));
    }

    return dist;
}

int32_t LastFrameFromSegmentLength(const std::vector<double> &dist,
                                   int32_t first_frame,
                                   double len) {
    for (uint32_t i = first_frame; i < dist.size(); i++) {
        if (dist[i] > dist[first_frame] + len) {
            return i;
        }
    }
    return -1;
}

double RotationError(const Eigen::Matrix4d &pose_error) {
    double a = pose_error(0, 0);
    double b = pose_error(1, 1);
    double c = pose_error(2, 2);
    double d = 0.5 * (a + b + c - 1.0);
    return std::acos(std::max(std::min(d, 1.0), -1.0));
}

double TranslationError(const Eigen::Matrix4d &pose_error) {
    double dx = pose_error(0, 3);
    double dy = pose_error(1, 3);
    double dz = pose_error(2, 3);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<errors> CalcSequenceErrors(const std::vector<Eigen::Matrix4d> &poses_gt,
                                       const std::vector<Eigen::Matrix4d> &poses_result) {
    // error vector
    std::vector<errors> err;

    // parameters
    int32_t step_size = 10;  // every second

    // pre-compute distances (from ground truth as reference)
    std::vector<double> dist = TrajectoryDistances(poses_gt);

    // for all start positions do
    for (uint32_t first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) {
        // for all segment lengths do
        for (int32_t i = 0; i < num_lengths; i++) {
            // current length
            double len = lengths[i];

            // compute last frame
            int32_t last_frame = LastFrameFromSegmentLength(dist, first_frame, len);

            // continue, if sequence not long enough
            if (last_frame == -1) {
                continue;
            }

            // compute rotational and translational errors
            Eigen::Matrix4d pose_delta_gt = poses_gt[first_frame].inverse() * poses_gt[last_frame];
            Eigen::Matrix4d pose_delta_result =
                poses_result[first_frame].inverse() * poses_result[last_frame];
            Eigen::Matrix4d pose_error = pose_delta_result.inverse() * pose_delta_gt;
            double r_err = RotationError(pose_error);
            double t_err = TranslationError(pose_error);

            // compute speed
            auto num_frames = static_cast<double>(last_frame - first_frame + 1);
            double speed = len / (0.1 * num_frames);

            // write to file
            err.emplace_back(first_frame, r_err / len, t_err / len, len, speed);
        }
    }

    // return error vector
    return err;
}
}  // namespace

namespace kiss_icp::metrics {

std::tuple<float, float> SeqError(const std::vector<Eigen::Matrix4d> &poses_gt,
                                  const std::vector<Eigen::Matrix4d> &poses_result) {
    std::vector<errors> err = CalcSequenceErrors(poses_gt, poses_result);
    double t_err = 0;
    double r_err = 0;

    for (const auto &it : err) {
        t_err += it.t_err;
        r_err += it.r_err;
    }

    double avg_trans_error = 100.0 * (t_err / static_cast<double>(err.size()));
    double avg_rot_error = (r_err / static_cast<double>(err.size())) / 3.14 * 180.0;

    return std::make_tuple(avg_trans_error, avg_rot_error);
}

std::tuple<float, float> AbsoluteTrajectoryError(const std::vector<Eigen::Matrix4d> &poses_gt,
                                                 const std::vector<Eigen::Matrix4d> &poses_result) {
    assert(poses_gt.size() == poses_result.size() &&
           "AbsoluteTrajectoryError| Different number of poses in ground truth and estimate");
    Eigen::MatrixXd source(3, poses_gt.size());
    Eigen::MatrixXd target(3, poses_gt.size());
    const size_t num_poses = poses_gt.size();
    // Align the two trajectories using SVD-ICP (Umeyama algorithm)
    for (size_t i = 0; i < num_poses; ++i) {
        source.block<3, 1>(0, i) = poses_result[i].block<3, 1>(0, 3);
        target.block<3, 1>(0, i) = poses_gt[i].block<3, 1>(0, 3);
    }
    const Eigen::Matrix4d T_align_trajectories = Eigen::umeyama(source, target, false);
    // ATE computation
    double ATE_rot = 0, ATE_trans = 0;
    for (size_t j = 0; j < num_poses; ++j) {
        // Apply alignement matrix
        const Eigen::Matrix4d T_estimate = T_align_trajectories * poses_result[j];
        const Eigen::Matrix4d T_ground_truth = poses_gt[j];
        const auto delta_T = T_estimate.inverse() * T_ground_truth;
        // Get angular error
        const double delta_theta = Eigen::AngleAxisd(delta_T.block<3, 3>(0, 0)).angle();
        const auto delta_trans = delta_T.block<3, 1>(0, 3);
        // Sum of Squares
        ATE_rot += delta_theta * delta_theta;
        ATE_trans += delta_trans.squaredNorm();
    }
    // Get the RMSE
    ATE_rot /= static_cast<double>(num_poses);
    ATE_trans /= static_cast<double>(num_poses);
    return std::make_tuple(std::sqrt(ATE_rot), std::sqrt(ATE_trans));
}
}  // namespace kiss_icp::metrics
