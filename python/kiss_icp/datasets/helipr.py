# MIT License
#
# Copyright (c) 2024 Benedikt Mersch, Saurabh Gupta, Ignacio Vizzo,
# Tiziano Guadagnino, Cyrill Stachniss.
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
import os
from pathlib import Path

import numpy as np
import open3d as o3d


class HeLiPRDataset:
    def __init__(self, data_dir: Path, sequence: str, *_, **__):
        self.sequence_id = sequence
        self.sequence_dir = os.path.join(data_dir, "LiDAR", self.sequence_id)
        self.scan_files = sorted(glob.glob(self.sequence_dir + "/*.ply"))
        self.scan_timestamps = [int(Path(file).stem) for file in self.scan_files]

        self.gt_file = os.path.join(data_dir, "LiDAR_GT", f"global_{self.sequence_id}_gt.txt")
        self.gt_poses = self.load_poses(self.gt_file)

        if len(self.scan_files) == 0:
            raise ValueError(f"Tried to read point cloud files in {data_dir} but none found")

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        file_path = self.scan_files[idx]
        pcd = o3d.t.io.read_point_cloud(file_path)
        points = pcd.point.positions.numpy()
        if self.sequence_id == "Aeva":
            return points, np.array([])
        try:
            timestamps = pcd.point.timestamps.numpy()
        except KeyError:
            timestamps = np.array([])

        return points, timestamps

    def load_poses(self, poses_file):
        from pyquaternion import Quaternion

        poses = np.loadtxt(poses_file, delimiter=" ")

        xyz = poses[:, 1:4]
        rotations = np.array(
            [Quaternion(x=x, y=y, z=z, w=w).rotation_matrix for x, y, z, w in poses[:, 4:]]
        )
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(self.__len__(), axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz

        return poses
