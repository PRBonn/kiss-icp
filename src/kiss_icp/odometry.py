# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import numpy as np

from kiss_icp.config import KISSConfig
from kiss_icp.deskew import MotionCompensator, StubCompensator
from kiss_icp.mapping import VoxelHashMap
from kiss_icp.preprocess import Preprocessor, Stubcessor
from kiss_icp.voxelization import voxel_down_sample


class Odometry:
    def __init__(self, config: KISSConfig, deskew: bool = False):
        self.config = config
        self.poses = []

        # Deskewing configuration
        self.compensator = MotionCompensator(self.config) if deskew else StubCompensator()
        # Bounding Box and PointCloud PreProcessing
        self.preprocess = Preprocessor(self.config) if self.config.data.preprocess else Stubcessor()

        # Use specified voxel size or compute one using the max range
        self.voxel_size = (
            self.config.mapping.voxel_size
            if hasattr(self.config.mapping, "voxel_size")
            else float(self.config.data.max_range / 100.0)
        )

        # Initialize VoxelHashMap, can be easily replaced with VDBs, Octrees, KD-Trees, etc.
        self.local_map = VoxelHashMap(
            voxel_size=self.voxel_size,
            max_distance=self.config.data.max_range,
            max_points_per_voxel=self.config.mapping.max_points_per_voxel,
        )

        # Adaptive thresholding options
        self.use_adaptive_threshold = not hasattr(self.config.adaptive_threshold, "fixed_threshold")
        self.initial_threshold = self.config.adaptive_threshold.initial_threshold
        self.model_deviation = np.eye(4)
        self.model_error_sse2 = 0
        self.num_samples = 0
        self.has_moved = False

    def register_frame(self, frame, timestamps):
        # Apply motion compensation
        frame = self.compensator.deskew_scan(frame, self.poses, timestamps)

        # Preprocess the input cloud
        frame = self.preprocess(frame)

        # Voxelize
        source, frame_downsample = self.voxelize(frame)

        # Get motion prediction and adaptive_threshold
        sigma = self._get_adaptive_threshold()
        prediction = self._get_prediction_model()

        # Compute initial_guess for ICP
        last_pose = self.poses[-1] if self.poses else np.eye(4)
        initial_guess = last_pose @ prediction

        new_pose = self.local_map.register_frame(
            points=source,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma / 3,
        )

        self.model_deviation = np.linalg.inv(initial_guess) @ new_pose
        self.local_map.add_points(frame_downsample, new_pose)
        self.poses.append(new_pose)
        return frame, source

    def voxelize(self, iframe):
        frame_downsample = voxel_down_sample(iframe, self.voxel_size * 0.5)
        source = voxel_down_sample(frame_downsample, self.voxel_size * 1.5)
        return source, frame_downsample

    def _get_adaptive_threshold(self):
        if not self.use_adaptive_threshold or self._has_not_moved():
            return self.initial_threshold

        model_error = self._compute_model_error()
        if model_error > self.config.adaptive_threshold.min_motion_th:
            self.model_error_sse2 += model_error**2
            self.num_samples += 1
        return (
            np.sqrt(self.model_error_sse2 / self.num_samples)
            if self.num_samples
            else self.initial_threshold
        )

    def _compute_model_error(self):
        Delta_t = self.model_deviation
        theta = np.arccos(0.5 * (np.trace(Delta_t[:3, :3]) - 1))
        delta_rot = 2 * self.config.data.max_range * np.sin(theta / 2.0)
        delta_trans = np.linalg.norm(Delta_t[:3, -1])
        return delta_trans + delta_rot

    def _get_prediction_model(self):
        if len(self.poses) < 2:
            return np.eye(4)
        return np.linalg.inv(self.poses[-2]) @ self.poses[-1]

    def _has_not_moved(self):
        return not self._has_moved()

    def _has_moved(self):
        if len(self.poses) > 1 and not self.has_moved:
            self.has_moved = (
                self._comptue_motion(self.poses[0], self.poses[-1])
                > 5 * self.config.adaptive_threshold.min_motion_th
            )
        return self.has_moved

    @staticmethod
    def _comptue_motion(T1, T2):
        return np.linalg.norm((np.linalg.inv(T1) @ T2)[:3, -1])
