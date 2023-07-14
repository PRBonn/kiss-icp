# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to mse, copy, modify, merge, publish, distribute, sublicense, and/or sell
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
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE msE OR OTHER DEALINGS IN THE
# SOFTWARE.
import glob
import os

import numpy as np


class KITTIOdometryDataset:
    def __init__(self, data_dir, sequence: int, *_, **__):
        self.sequence_id = str(int(sequence)).zfill(2)
        self.kitti_sequence_dir = os.path.join(data_dir, "sequences", self.sequence_id)
        self.velodyne_dir = os.path.join(self.kitti_sequence_dir, "velodyne/")

        self.scan_files = sorted(glob.glob(self.velodyne_dir + "*.bin"))
        self.calibration = self.read_calib_file(os.path.join(self.kitti_sequence_dir, "calib.txt"))

        # Load GT Poses (if available)
        if sequence < 11:
            self.poses_fn = os.path.join(data_dir, f"poses/{self.sequence_id}.txt")
            self.gt_poses = self.load_poses(self.poses_fn)

        # Add correction for KITTI datasets, can be easilty removed if unwanted
        from kiss_icp.pybind import kiss_icp_pybind

        self.correct_kitti_scan = lambda frame: np.asarray(
            kiss_icp_pybind._correct_kitti_scan(kiss_icp_pybind._Vector3dVector(frame))
        )

    def __getitem__(self, idx):
        return self.scans(idx)

    def __len__(self):
        return len(self.scan_files)

    def scans(self, idx):
        return self.read_point_cloud(self.scan_files[idx])

    def apply_calibration(self, poses: np.ndarray) -> np.ndarray:
        """Converts from Velodyne to Camera Frame"""
        Tr = np.eye(4, dtype=np.float64)
        Tr[:3, :4] = self.calibration["Tr"].reshape(3, 4)
        return Tr @ poses @ np.linalg.inv(Tr)

    def read_point_cloud(self, scan_file: str):
        points = np.fromfile(scan_file, dtype=np.float32).reshape((-1, 4))[:, :3].astype(np.float64)
        #  points = points[points[:, 2] > -2.9]  # Remove the annoying reflections
        points = self.correct_kitti_scan(points)
        return points

    def load_poses(self, poses_file):
        def _lidar_pose_gt(poses_gt):
            _tr = self.calibration["Tr"].reshape(3, 4)
            tr = np.eye(4, dtype=np.float64)
            tr[:3, :4] = _tr
            left = np.einsum("...ij,...jk->...ik", np.linalg.inv(tr), poses_gt)
            right = np.einsum("...ij,...jk->...ik", left, tr)
            return right

        poses = np.loadtxt(poses_file, delimiter=" ")
        n = poses.shape[0]
        poses = np.concatenate(
            (poses, np.zeros((n, 3), dtype=np.float32), np.ones((n, 1), dtype=np.float32)), axis=1
        )
        poses = poses.reshape((n, 4, 4))  # [N, 4, 4]
        return _lidar_pose_gt(poses)

    def get_frames_timestamps(self) -> np.ndarray:
        timestamps = np.loadtxt(os.path.join(self.kitti_sequence_dir, "times.txt")).reshape(-1, 1)
        return timestamps

    @staticmethod
    def read_calib_file(file_path: str) -> dict:
        calib_dict = {}
        with open(file_path, "r") as calib_file:
            for line in calib_file.readlines():
                tokens = line.split(" ")
                if tokens[0] == "calib_time:":
                    continue
                # Only read with float data
                if len(tokens) > 0:
                    values = [float(token) for token in tokens[1:]]
                    values = np.array(values, dtype=np.float32)

                    # The format in KITTI's file is <key>: <f1> <f2> <f3> ...\n -> Remove the ':'
                    key = tokens[0][:-1]
                    calib_dict[key] = values
        return calib_dict
