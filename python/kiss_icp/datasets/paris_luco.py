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
import os
from pathlib import Path

import numpy as np
from plyfile import PlyData


class ParisLucoDataset:
    def __init__(self, data_dir: Path, *_, **__):
        # Config stuff
        self.sequence_id = os.path.basename(data_dir)
        self.sequence_dir = os.path.realpath(data_dir)
        self.velodyne_dir = os.path.join(self.sequence_dir, "frames/")
        self.scan_files = sorted(glob.glob(self.velodyne_dir + "*.ply"))
        self.gt_poses = self.load_gt_poses(os.path.join(self.sequence_dir, "gt_traj_lidar.txt"))

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(self.scan_files[idx])

    def read_point_cloud(self, file_path: str):
        plydata = PlyData.read(file_path)
        x = np.asarray(plydata.elements[0].data["x"]).reshape(-1, 1)
        y = np.asarray(plydata.elements[0].data["y"]).reshape(-1, 1)
        z = np.asarray(plydata.elements[0].data["z"]).reshape(-1, 1)
        points = np.concatenate([x, y, z], axis=1)
        timestamps = np.asarray(plydata.elements[0].data["timestamp"])
        timestamps = timestamps / np.max(timestamps)
        return points.astype(np.float64), timestamps

    def load_gt_poses(self, file_path):
        gt_poses = []
        for xyz in np.loadtxt(file_path):
            T = np.eye(4)
            T[:3, 3] = xyz
            gt_poses.append(T)
        return np.array(gt_poses).reshape(-1, 4, 4)

    def apply_calibration(self, poses):
        """ParisLucoDataset only has a x, y, z trajectory, so we must will em all"""
        new_poses = poses.copy()
        new_poses[:, :3, :3] = np.eye(3)
        return new_poses
