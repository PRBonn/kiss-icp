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

#include "kiss_icp/Deskew.hpp"
#include "kiss_icp/Map.hpp"
#include "kiss_icp/Preprocessing.hpp"
#include "kiss_icp/VoxelHashMap.hpp"
#include "metrics/Metrics.hpp"
#include "stl_vector_eigen.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

namespace kiss_icp {
PYBIND11_MODULE(kiss_icp_pybind, m) {
    auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors_double<Eigen::Vector3d>);

    using MapType = Map<VoxelHashMap>;
    py::class_<MapType> internal_map(m, "_VoxelHashMap", "Don't use this");
    internal_map
        .def(py::init<double, double, int>(), "voxel_size"_a, "max_distance"_a,
             "max_points_per_voxel"_a)
        .def("_clear", &MapType::Clear)
        .def("_empty", &MapType::Empty)
        .def("_add_points",
             py::overload_cast<const MapType::Vector3dVector&, const Eigen::Vector3d&>(
                 &MapType::AddPoints),
             "points"_a, "origin"_a)
        .def("_add_points",
             py::overload_cast<const MapType::Vector3dVector&, const Eigen::Matrix4d&>(
                 &MapType::AddPoints),
             "points"_a, "pose"_a)
        .def("_point_cloud", &MapType::Pointcloud)
        .def("_get_correspondences", &MapType::GetCorrespondences, "points"_a,
             "max_correspondance_distance"_a)
        .def("_register_point_cloud", &MapType::RegisterPoinCloud, "points"_a, "initial_guess"_a,
             "max_correspondance_distance"_a, "kernel"_a);

    // Floating functions, unrelated to the mapping class
    m.def("_voxel_down_sample", &VoxelDownsample, "frame"_a, "voxel_size"_a);
    // KITTI utils
    m.def("_kitti_seq_error", &SeqError, "gt_poses"_a, "results_poses"_a);
    m.def("_absolute_trajectory_error", &AbsoluteTrajectoryError, "gt_poses"_a, "results_poses"_a);
    // DeSkewScan
    m.def("_velocity_estimation", &VelocityEstimation, "start_pose"_a, "finish_pose"_a,
          "scan_duration"_a);
    m.def("_deskew_scan", &DeSkewScan, "frame"_a, "timestamps"_a, "linear_velocity"_a,
          "angular_velocity"_a);
    m.def("_preprocess", &Preprocess, "frame"_a, "max_range"_a, "min_range"_a);
    m.def("_correct_kitti_scan", &CorrectKITTIScan, "frame"_a);
}

}  // namespace kiss_icp
