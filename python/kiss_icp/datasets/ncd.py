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
import importlib
import os
import re
from pathlib import Path

import numpy as np
from pyquaternion import Quaternion


class NewerCollegeDataset:
    def __init__(self, data_dir: Path, *_, **__):
        try:
            self.PyntCloud = importlib.import_module("pyntcloud").PyntCloud
        except ModuleNotFoundError:
            print(f'Newer College requires pnytccloud: "pip install pyntcloud"')

        self.data_source = os.path.join(data_dir, "")
        self.scan_folder = os.path.join(self.data_source, "raw_format/ouster_scan")
        self.pose_file = os.path.join(self.data_source, "ground_truth/registered_poses.csv")
        self.sequence_id = os.path.basename(data_dir)

        # Load scan files and poses
        self.scan_files = self.get_pcd_filenames(self.scan_folder)
        self.gt_poses = self.load_gt_poses(self.pose_file)

        # Visualization Options
        self.use_global_visualizer = True

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        file_path = os.path.join(self.scan_folder, self.scan_files[idx])
        return self.getitem(file_path)

    def getitem(self, scan_file: str):
        points = self.PyntCloud.from_file(scan_file).points[["x", "y", "z"]].to_numpy()
        timestamps = self.get_timestamps()
        if points.shape[0] != timestamps.shape[0]:
            # MuRan has some broken point clouds, just fallback to no timestamps
            return points.astype(np.float64), np.ones(points.shape[0])
        return points.astype(np.float64), timestamps

    @staticmethod
    def get_timestamps():
        H = 64
        W = 1024
        return (np.floor(np.arange(H * W) / H) / W).reshape(-1, 1)

    @staticmethod
    def get_pcd_filenames(scans_folder):
        # cloud_1583836591_182590976.pcd
        regex = re.compile("^cloud_(\d*_\d*)")

        def get_cloud_timestamp(pcd_filename):
            m = regex.search(pcd_filename)
            secs, nsecs = m.groups()[0].split("_")
            return int(secs) * int(1e9) + int(nsecs)

        return sorted(os.listdir(scans_folder), key=get_cloud_timestamp)

    @staticmethod
    def load_gt_poses(file_path: str):
        """Taken from pyLiDAR-SLAM/blob/master/slam/dataset/nhcd_dataset.py"""
        ground_truth_df = np.genfromtxt(str(file_path), delimiter=",", dtype=np.float64)
        xyz = ground_truth_df[:, 2:5]
        rotations = np.array(
            [
                Quaternion(x=x, y=y, z=z, w=w).rotation_matrix
                for x, y, z, w in ground_truth_df[:, 5:]
            ]
        )

        num_poses = rotations.shape[0]
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(num_poses, axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz

        T_CL = np.eye(4, dtype=np.float32)
        T_CL[:3, :3] = Quaternion(x=0, y=0, z=0.924, w=0.383).rotation_matrix
        T_CL[:3, 3] = np.array([-0.084, -0.025, 0.050], dtype=np.float32)
        poses = np.einsum("nij,jk->nik", poses, T_CL)
        poses = np.einsum("ij,njk->nik", np.linalg.inv(poses[0]), poses)
        return poses
