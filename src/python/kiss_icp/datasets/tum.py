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
from pathlib import Path

import numpy as np


class TUMDataset:
    def __init__(self, data_dir: Path, *_, **__):
        try:
            self.o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as err:
            print(f'open3d is not installed on your system, run "pip install open3d"')
            exit(1)

        self.data_dir = data_dir
        self.sequence_id = os.path.basename(data_dir)

        # Load depth frames
        self.depth_frames = np.loadtxt(fname=self.data_dir / "depth.txt", dtype=str)

        # rgb single frame
        rgb_path = os.path.join(self.data_dir, "rgb", os.listdir(self.data_dir / "rgb")[0])
        self.rgb_default_frame = self.o3d.io.read_image(rgb_path)

        # Load GT poses
        self.gt_list = np.loadtxt(fname=self.data_dir / "groundtruth.txt", dtype=str)
        self.gt_poses = self.load_poses(self.gt_list)

    def __len__(self):
        return len(self.depth_frames)

    def find_closest_ts(self, ts):
        times = self.gt_list[:, 0]
        diff = np.abs(times.astype(np.float64) - float(ts))
        idx = diff.argmin()
        return idx

    def load_poses(self, gt_list):
        def conver_to_homo(v):
            x, y, z = v[0], v[1], v[2]
            qx, qy, qz, qw = v[3], v[4], v[5], v[6]
            T = np.eye(4)
            T[:3, -1] = np.array([x, y, z])
            T[:3, :3] = self.o3d.geometry.Geometry3D.get_rotation_matrix_from_quaternion(
                np.array([qw, qx, qy, qz])
            )
            return T

        poses = []
        for depth_id, _ in self.depth_frames:
            pose_timestamp = self.find_closest_ts(depth_id)
            pose = conver_to_homo(gt_list[pose_timestamp][1:])
            poses.append(pose)
        return np.asarray(poses)

    def get_time_stamps(self):
        return self.depth_frames[:, 0]

    def get_gt_time_stamps(self):
        return self.gt_list[:, 0]

    def __getitem__(self, idx):
        depth_id = self.depth_frames[idx][-1]
        depth_raw = self.o3d.io.read_image(str(self.data_dir / depth_id))
        rgbd_image = self.o3d.geometry.RGBDImage.create_from_tum_format(
            self.rgb_default_frame, depth_raw
        )
        pcd = self.o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            self.o3d.camera.PinholeCameraIntrinsic(
                self.o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
        )
        return np.array(pcd.points, dtype=np.float64)
