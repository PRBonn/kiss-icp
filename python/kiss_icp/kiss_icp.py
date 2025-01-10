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
from datetime import time
import numpy as np

from kiss_icp.config import KISSConfig
from kiss_icp.pybind import kiss_icp_pybind
from kiss_icp.voxelization import voxel_down_sample


class KissICP:
    def __init__(self, config: KISSConfig):
        self.last_delta = np.eye(4)
        self.config = config
        self.kiss_icp = kiss_icp_pybind._KissICP(config)

    @property
    def last_pose(self):
        self.kiss_icp._pose()

    def register_frame(self, frame, timestamps):
        old_pose = self.last_pose
        frame, source = self.kiss_icp._register_frame(frame, timestamps)
        self.last_delta = np.linalg.inv(old_pose) @ self.last_pose
        return np.asarray(frame), np.asarray(source)

    def voxelize(self, iframe):
        frame_downsample = voxel_down_sample(iframe, self.config.mapping.voxel_size * 0.5)
        source = voxel_down_sample(frame_downsample, self.config.mapping.voxel_size * 1.5)
        return source, frame_downsample
