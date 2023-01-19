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
from kiss_icp.pybind import kiss_icp_pybind


def get_motion_compensator(config: KISSConfig):
    return MotionCompensator(config) if config.data.deskew else StubCompensator()


class StubCompensator:
    def deskew_scan(self, frame, poses, timestamps):
        return frame


class MotionCompensator:
    def __init__(self, config: KISSConfig):
        self.scan_duration = 1 / config.data.lidar_frequency
        self.mid_pose_timestamp = 0.5  # TODO: Expose this

    # This could be an IMU estimation
    def velocity_estimation(self, poses):
        return kiss_icp_pybind._velocity_estimation(
            start_pose=poses[-2],
            finish_pose=poses[-1],
            scan_duration=self.scan_duration,
        )

    def deskew_scan(self, frame, poses, timestamps):
        if len(poses) < 2:
            return frame

        linear_velocity, angular_velocity = self.velocity_estimation(poses)
        return np.asarray(
            kiss_icp_pybind._deskew_scan(
                frame=kiss_icp_pybind._Vector3dVector(frame),
                timestamps=self.scan_duration * (timestamps - self.mid_pose_timestamp),
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
            )
        )
