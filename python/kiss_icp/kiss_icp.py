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
from kiss_icp.deskew import get_motion_compensator
from kiss_icp.mapping import get_voxel_hash_map
from kiss_icp.preprocess import get_preprocessor
from kiss_icp.registration import Estimate, get_registration
from kiss_icp.threshold import get_threshold_estimator
from kiss_icp.voxelization import voxel_down_sample


class KissICP:
    def __init__(self, config: KISSConfig):
        self.estimates = []
        self.config = config
        self.compensator = get_motion_compensator(config)
        self.adaptive_threshold = get_threshold_estimator(self.config)
        self.registration = get_registration(self.config)
        self.local_map = get_voxel_hash_map(self.config)
        self.preprocess = get_preprocessor(self.config)

    def register_frame(self, frame, timestamps):
        # Apply motion compensation
        frame = self.compensator.deskew_scan(frame, self.estimates, timestamps)

        # Preprocess the input cloud
        frame = self.preprocess(frame)

        # Voxelize
        source, frame_downsample = self.voxelize(frame)

        # Get motion prediction and adaptive_threshold
        sigma = self.get_adaptive_threshold()

        # Compute initial_guess for ICP
        prediction = self.get_prediction_model()
        initial_guess = self.estimates[-1] if self.estimates else Estimate()
        initial_guess = initial_guess @ prediction

        # Run ICP
        new_estimate = self.registration.align_points_to_map(
            points=source,
            voxel_map=self.local_map,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma / 3,
        )

        pose_deviation = (initial_guess.inverse() @ new_estimate).pose
        self.adaptive_threshold.update_model_deviation(pose_deviation)
        self.local_map.update(frame_downsample, new_estimate.pose)
        self.estimates.append(new_estimate)
        return frame, source

    def voxelize(self, iframe):
        frame_downsample = voxel_down_sample(iframe, self.config.mapping.voxel_size * 0.5)
        source = voxel_down_sample(frame_downsample, self.config.mapping.voxel_size * 1.5)
        return source, frame_downsample

    def get_adaptive_threshold(self):
        return (
            self.config.adaptive_threshold.initial_threshold
            if not self.has_moved()
            else self.adaptive_threshold.get_threshold()
        )

    def get_prediction_model(self):
        prediction = Estimate()
        if len(self.estimates) > 1:
            pose_from = self.estimates[-2].pose
            pose_to = self.estimates[-1].pose
            prediction.pose = np.linalg.inv(pose_from) @ pose_to
        return prediction

    def has_moved(self):
        if len(self.estimates) < 1:
            return False
        compute_motion = lambda T1, T2: np.linalg.norm((T1.inverse() @ T2).pose[:3, -1])
        motion = compute_motion(self.estimates[0], self.estimates[-1])
        return motion > 5 * self.config.adaptive_threshold.min_motion_th
