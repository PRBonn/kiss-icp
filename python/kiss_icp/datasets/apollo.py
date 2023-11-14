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
import glob
import importlib
import os
import sys
from pathlib import Path

import natsort
import numpy as np
from pyquaternion import Quaternion


class ApolloDataset:
    def __init__(self, data_dir: Path, *_, **__):
        try:
            self.o3d = importlib.import_module("open3d")
        except ModuleNotFoundError:
            print(
                'pcd files requires open3d and is not installed on your system run "pip install open3d"'
            )
            sys.exit(1)

        self.scan_files = natsort.natsorted(glob.glob(f"{data_dir}/pcds/*.pcd"))
        self.gt_poses = self.read_poses(f"{data_dir}/poses/gt_poses.txt")
        self.sequence_id = os.path.basename(data_dir)

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.get_scan(self.scan_files[idx])

    def get_scan(self, scan_file: str):
        points = np.asarray(self.o3d.io.read_point_cloud(scan_file).points, dtype=np.float64)
        return points.astype(np.float64)

    @staticmethod
    def read_poses(file):
        data = np.loadtxt(file)
        _, _, translations, qxyzw = np.split(data, [1, 2, 5], axis=1)
        rotations = np.array(
            [Quaternion(x=x, y=y, z=z, w=w).rotation_matrix for x, y, z, w in qxyzw]
        )
        poses = np.zeros([rotations.shape[0], 4, 4])
        poses[:, :3, -1] = translations
        poses[:, :3, :3] = rotations
        poses[:, -1, -1] = 1
        # Convert from global coordinate poses to local poses
        first_pose = poses[0, :, :]
        return np.linalg.inv(first_pose) @ poses
