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
#include "metrics.hpp"

#include <Eigen/Core>
#include <tuple>

std::tuple<float, float> SeqError(const std::vector<Eigen::Matrix4d>& poses_gt,
                                  const std::vector<Eigen::Matrix4d>& poses_result) {
    std::vector<errors> err = calcSequenceErrors(poses_gt, poses_result);
    double t_err = 0;
    double r_err = 0;

    for (const auto& it : err) {
        t_err += it.t_err;
        r_err += it.r_err;
    }

    double avg_trans_error = avg_trans_error = 100.0 * (t_err / float(err.size()));
    double avg_rot_error = avg_rot_error = 100.0 * (r_err / float(err.size())) / 3.14 * 180.0;

    return std::make_tuple(avg_trans_error, avg_rot_error);
}
std::tuple<float, float> AbsoluteTrajectoryError(const std::vector<Eigen::Matrix4d>& poses_gt,
                                                 const std::vector<Eigen::Matrix4d>& poses_result) {
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
        const Eigen::Matrix4d& T_ground_truth = poses_gt[j];
        // Compute error in translation and rotation matrix (ungly)
        const Eigen::Matrix3d delta_R =
            T_ground_truth.block<3, 3>(0, 0) * T_estimate.block<3, 3>(0, 0).transpose();
        const Eigen::Vector3d delta_t =
            T_ground_truth.block<3, 1>(0, 3) - delta_R * T_estimate.block<3, 1>(0, 3);
        // Get angular error
        const double theta = Eigen::AngleAxisd(delta_R).angle();
        // Sum of Squares
        ATE_rot += theta * theta;
        ATE_trans += delta_t.squaredNorm();
    }
    // Get the RMSE
    ATE_rot /= num_poses;
    ATE_trans /= num_poses;
    return std::make_tuple(std::sqrt(ATE_rot), std::sqrt(ATE_trans));
}
