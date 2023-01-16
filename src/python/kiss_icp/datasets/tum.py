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
"""Original Implementation done by Federico Magistri"""
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
        self.rgb_txt = os.path.join(self.data_dir, "rgb.txt")
        self.depth_txt = os.path.join(self.data_dir, "depth.txt")
        self.sequence_id = os.path.basename(data_dir)

        # TODO(Nacho): Speed up this extra time consuming operation!
        print("Running association, this will take some time...")
        rgb_list = self.read_file_list(self.rgb_txt)
        depth_list = self.read_file_list(self.depth_txt)
        self.matches = self.associate(rgb_list, depth_list)
        print("Done!")

        # Load GT poses
        print("Loading GT poses, this will also take some time!")
        self.gt_list = self.read_gt_list(os.path.join(self.data_dir, "groundtruth.txt"))
        # TODO(Nacho): Apply calibration here, since evluation is entierly broken
        self.gt_poses = self.load_poses()
        print("Done!")

    def __len__(self):
        return len(self.matches)

    def associate(self, first_keys, second_keys):
        """Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we
        aim to find the closest match for every input tuple.

        Input:
        first_list -- first dictionary of (stamp,data) tuples
        second_list -- second dictionary of (stamp,data) tuples
        offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
        max_difference -- search radius for candidate generation

        Output:
        matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
        """
        offset = 0.0
        max_difference = 0.2
        potential_matches = [
            (abs(a - (b + offset)), a, b)
            for a in first_keys.keys()
            for a in first_keys.keys()
            for b in second_keys.keys()
            if abs(a - (b + offset)) < max_difference
        ]
        potential_matches.sort()
        matches = []
        for diff, a, b in potential_matches:
            if a in first_keys and b in second_keys:
                first_keys.pop(a)
                second_keys.pop(b)
                matches.append((a, b))

        matches.sort()
        return matches

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
        for _, depth_id in self.matches:
            pose_timestamp = self.find_closest_ts(depth_id)
            pose = conver_to_homo(self.gt_list[pose_timestamp])
            poses.append(pose)
        return np.asarray(poses)

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
        file = open(filename)
        data = file.read()
        lines = data.replace(",", " ").replace("\t", " ").split("\n")
        # lines = lines[:100]
        list = [
            [v.strip() for v in line.split(" ") if v.strip() != ""]
            for line in lines
            if len(line) > 0 and line[0] != "#"
        ]
        list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
        return dict(list)

    def read_gt_list(self, filename):
        """Reads a trajectory from a text file.

        File format:
        The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
        and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

        Input:
        filename -- File name

        Output:
        dict -- dictionary of (stamp,data) tuples
        """
        file = open(filename)
        data = file.read()
        lines = data.replace(",", " ").replace("\t", " ").split("\n")
        # lines = lines[:100]
        list = [
            [v.strip() for v in line.split(" ") if v.strip() != ""]
            for line in lines
            if len(line) > 0 and line[0] != "#"
        ]
        list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
        return dict(list)

    def __getitem__(self, idx):
        rgb_id, depth_id = self.matches[idx]
        rgb_path = os.path.join(self.data_dir, "rgb", "{:.6f}".format(rgb_id) + ".png")
        depth_path = os.path.join(self.data_dir, "depth", "{:.6f}".format(depth_id) + ".png")
        color_raw = self.o3d.io.read_image(rgb_path)
        depth_raw = self.o3d.io.read_image(depth_path)
        rgbd_image = self.o3d.geometry.RGBDImage.create_from_tum_format(color_raw, depth_raw)
        pcd = self.o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            self.o3d.camera.PinholeCameraIntrinsic(
                self.o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            ),
        )
        # TODO(Nacho): Make sure this transformation is also applied to the gt_poses
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        return np.array(pcd.points, dtype=np.float64)
