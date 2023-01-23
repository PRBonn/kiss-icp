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
        # TODO(Nacho): Remove Open3D dependency
        try:
            self.o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as err:
            print(f'open3d is not installed on your system, run "pip install open3d"')
            exit(1)

        self.data_dir = data_dir
        self.sequence_id = os.path.basename(data_dir)

        # Load depth frames
        self.depth_list = self.read_file_list((self.data_dir / "depth.txt"))
        self.depth_frames = list(self.depth_list.keys())

        # rgb single frame
        rgb_path = os.path.join(self.data_dir, "rgb", os.listdir(self.data_dir / "rgb")[0])
        self.rgb_default_frame = self.o3d.io.read_image(rgb_path)

        # Load GT poses
        self.gt_list = self.read_file_list(os.path.join(self.data_dir, "groundtruth.txt"))
        self.gt_poses = self.load_poses()

    def __len__(self):
        return len(self.depth_frames)

    def find_closest_ts(self, ts):
        times = list(self.gt_list.keys())
        diff = np.abs(np.array(times) - ts)
        idx = diff.argmin()
        return times[idx]

    def load_poses(self):
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
        for depth_id in self.depth_frames:
            pose_timestamp = self.find_closest_ts(depth_id)
            pose = conver_to_homo(self.gt_list[pose_timestamp])
            poses.append(pose)
        return np.asarray(poses)

    def get_time_stamps(self):
        return list(self.depth_list.keys())

    def get_gt_time_stamps(self):
        return list(self.gt_list.keys())

    def read_file_list(self, filename):
        """Reads a trajectory from a text file.

        File format:
        The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
        and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

        Input:
        filename -- File name

        Output:
        dict -- dictionary of (stamp,data) tuples
        """
        file_lines = np.loadtxt(fname=filename, dtype=str, comments="#", delimiter=" ")
        list = [(float(l[0]), l[1:]) for l in file_lines if len(l) > 1]
        return dict(list)

    def __getitem__(self, idx):
        depth_id = self.depth_frames[idx]
        depth_path = os.path.join(self.data_dir, "depth", "{:.6f}".format(depth_id) + ".png")
        depth_raw = self.o3d.io.read_image(depth_path)
        rgbd_image = self.o3d.geometry.RGBDImage.create_from_tum_format(
            self.rgb_default_frame, depth_raw
        )
        pcd = self.o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            self.o3d.camera.PinholeCameraIntrinsic(
                self.o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
        )
        pcd.transform([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        return np.array(pcd.points, dtype=np.float64)
