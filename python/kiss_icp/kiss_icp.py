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
from kiss_icp.registration import get_registration
from kiss_icp.threshold import get_threshold_estimator
from kiss_icp.voxelization import voxel_down_sample


class KissICP:
    def __init__(self, config: KISSConfig):
        self.last_pose = np.eye(4)
        self.last_delta = np.eye(4)
        self.config = config
        self.compensator = get_motion_compensator(config)
        self.adaptive_threshold = get_threshold_estimator(self.config)
        self.registration = get_registration(self.config)
        self.local_map = get_voxel_hash_map(self.config)
        self.preprocess = get_preprocessor(self.config)

    def register_frame(self, frame, timestamps):
        # Apply motion compensation
        frame = self.compensator.deskew_scan(frame, timestamps, self.last_delta)

        # Preprocess the input cloud
        frame = self.preprocess(frame)

        # Voxelize
        source, frame_downsample = self.voxelize(frame)

        # Get adaptive_threshold
        sigma = self.adaptive_threshold.get_threshold()

        # Compute initial_guess for ICP
        initial_guess = self.last_pose @ self.last_delta

        # Run ICP
        new_pose = self.registration.align_points_to_map(
            points=source,
            voxel_map=self.local_map,
            initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma,
            kernel=sigma / 3,
        )

        # Compute the difference between the prediction and the actual estimate
        model_deviation = np.linalg.inv(initial_guess) @ new_pose

        # Update step: threshold, local map, delta, and the last pose
        self.adaptive_threshold.update_model_deviation(model_deviation)
        self.local_map.update(frame_downsample, new_pose)
        self.last_delta = np.linalg.inv(self.last_pose) @ new_pose
        self.last_pose = new_pose

        # Return the (deskew) input raw scan (frame) and the points used for registration (source)
        return frame, source

    def voxelize(self, iframe):
        frame_downsample = voxel_down_sample(iframe, self.config.mapping.voxel_size * 0.5)
        source = voxel_down_sample(frame_downsample, self.config.mapping.voxel_size * 1.5)
        return source, frame_downsample
