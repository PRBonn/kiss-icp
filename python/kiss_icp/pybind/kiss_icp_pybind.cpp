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
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "kiss_icp/core/Deskew.hpp"
#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include "kiss_icp/metrics/Metrics.hpp"
#include "stl_vector_eigen.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

namespace kiss_icp {
PYBIND11_MODULE(kiss_icp_pybind, m) {
    auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors_double<Eigen::Vector3d>);

    // Map representation
    py::class_<VoxelHashMap> internal_map(m, "_VoxelHashMap", "Don't use this");
    internal_map
        .def(py::init<double, double, int>(), "voxel_size"_a, "max_distance"_a,
             "max_points_per_voxel"_a)
        .def("_clear", &VoxelHashMap::Clear)
        .def("_empty", &VoxelHashMap::Empty)
        .def("_update",
             py::overload_cast<const VoxelHashMap::Vector3dVector &, const Eigen::Vector3d &>(
                 &VoxelHashMap::Update),
             "points"_a, "origin"_a)
        .def(
            "_update",
            [](VoxelHashMap &self, const VoxelHashMap::Vector3dVector &points,
               const Eigen::Matrix4d &T) {
                Sophus::SE3d pose(T);
                self.Update(points, pose);
            },
            "points"_a, "pose"_a)
        .def("_point_cloud", &VoxelHashMap::Pointcloud)
        .def("_get_correspondences", &VoxelHashMap::GetCorrespondences, "points"_a,
             "max_correspondance_distance"_a);

    // Point Cloud registration
    m.def(
        "_register_point_cloud",
        [](const std::vector<Eigen::Vector3d> &points, const VoxelHashMap &voxel_map,
           const Eigen::Matrix4d &T_guess, double max_correspondence_distance, double kernel) {
            Sophus::SE3d initial_guess(T_guess);
            return RegisterFrame(points, voxel_map, initial_guess, max_correspondence_distance,
                                 kernel)
                .matrix();
        },
        "points"_a, "voxel_map"_a, "initial_guess"_a, "max_correspondance_distance"_a, "kernel"_a);
    // AdaptiveThreshold bindings
    py::class_<AdaptiveThreshold> adaptive_threshold(m, "_AdaptiveThreshold", "Don't use this");
    adaptive_threshold
        .def(py::init<double, double, double>(), "initial_threshold"_a, "min_motion_th"_a,
             "max_range"_a)
        .def("_compute_threshold", &AdaptiveThreshold::ComputeThreshold)
        .def(
            "_update_model_deviation",
            [](AdaptiveThreshold &self, const Eigen::Matrix4d &T) {
                Sophus::SE3d model_deviation(T);
                self.UpdateModelDeviation(model_deviation);
            },
            "model_deviation"_a);

    // DeSkewScan
    m.def(
        "_deskew_scan",
        [](const std::vector<Eigen::Vector3d> &frame, const std::vector<double> &timestamps,
           const Eigen::Matrix4d &T_start, const Eigen::Matrix4d &T_finish) {
            Sophus::SE3d start_pose(T_start);
            Sophus::SE3d finish_pose(T_finish);
            return DeSkewScan(frame, timestamps, start_pose, finish_pose);
        },
        "frame"_a, "timestamps"_a, "start_pose"_a, "finish_pose"_a);

    // prerpocessing modules
    m.def("_voxel_down_sample", &VoxelDownsample, "frame"_a, "voxel_size"_a);
    m.def("_preprocess", &Preprocess, "frame"_a, "max_range"_a, "min_range"_a);
    m.def("_correct_kitti_scan", &CorrectKITTIScan, "frame"_a);

    // Metrics
    m.def("_kitti_seq_error", &metrics::SeqError, "gt_poses"_a, "results_poses"_a);
    m.def("_absolute_trajectory_error", &metrics::AbsoluteTrajectoryError, "gt_poses"_a,
          "results_poses"_a);
}

}  // namespace kiss_icp
