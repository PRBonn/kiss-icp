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
import importlib
import os
import struct
import sys
from pathlib import Path

import natsort
import numpy as np
from pyquaternion import Quaternion

from kiss_icp.datasets import supported_file_extensions


class HeLiPRDataset:
    def __init__(self, data_dir: Path, sequence: str, *_, **__):
        try:
            self.o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as e:
            raise ModuleNotFoundError(
                "Open3D is not installed on your system, to fix this either "
                'run "pip install open3d" '
                "or check https://www.open3d.org/docs/release/getting_started.html"
            ) from e

        self.sequence_id = sequence
        self.sequence_dir = os.path.join(data_dir, "LiDAR", self.sequence_id)
        scan_files = os.listdir(self.sequence_dir)
        scan_timestamps = [int(Path(file).stem) for file in scan_files]

        pose_file = os.path.join(data_dir, "LiDAR_GT", f"{self.sequence_id}_gt.txt")
        pose_timestamps, poses = self.read_poses(pose_file)

        # Match number of scans with number of references poses available
        self.gt_poses = np.array(
            [pose for time, pose in zip(pose_timestamps, poses) if time in scan_timestamps]
        ).reshape(-1, 4, 4)

        scan_files = [file for file in scan_files if int(Path(file).stem) in pose_timestamps]

        self.scan_files = np.array(
            natsort.natsorted(
                [
                    os.path.join(self.sequence_dir, fn)
                    for fn in os.listdir(self.sequence_dir)
                    if any(fn.endswith(ext) for ext in supported_file_extensions())
                ]
            ),
            dtype=str,
        )
        if len(self.scan_files) == 0:
            raise ValueError(f"Tried to read point cloud files in {data_dir} but none found")

        # Obtain the pointcloud reader for the given data folder
        if self.sequence_id == "Avia":
            self.format_string = "fffBBBL"
            self.intensity_channel = None
            self.time_channel = 6
        elif self.sequence_id == "Aeva":
            self.format_string = "ffffflBf"
            self.format_string_no_intensity = "ffffflB"
            self.intensity_channel = 7
            self.time_channel = 5
        elif self.sequence_id == "Ouster":
            self.format_string = "ffffIHHH"
            self.intensity_channel = 3
            self.time_channel = 4
        elif self.sequence_id == "Velodyne":
            self.format_string = "ffffHf"
            self.intensity_channel = 3
            self.time_channel = 5
        else:
            print("[ERROR] Unsupported LiDAR Type")
            sys.exit()

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        data = self.get_data(idx)
        points = self.read_point_cloud(data)
        timestamps = self.read_timestamps(data)
        return points, timestamps

    def read_poses(self, pose_file: str):
        gt = np.loadtxt(pose_file, delimiter=" ")
        time = gt[:, 0]
        xyz = gt[:, 1:4]
        rotations = np.array(
            [Quaternion(x=x, y=y, z=z, w=w).rotation_matrix for x, y, z, w in gt[:, 4:]]
        )
        poses = np.eye(4, dtype=np.float64).reshape(1, 4, 4).repeat(len(gt), axis=0)
        poses[:, :3, :3] = rotations
        poses[:, :3, 3] = xyz
        return time, poses

    def get_data(self, idx: int):
        file_path = self.scan_files[idx]
        list_lines = []

        # Special case, see https://github.com/minwoo0611/HeLiPR-File-Player/blob/e8d95e390454ece1415ae9deb51515f63730c10a/src/ROSThread.cpp#L632
        if self.sequence_id == "Aeva" and int(Path(file_path).stem) <= 1691936557946849179:
            self.intensity_channel = None
            format_string = self.format_string_no_intensity
        else:
            format_string = self.format_string

        chunk_size = struct.calcsize(f"={format_string}")
        with open(file_path, "rb") as f:
            binary = f.read()
            offset = 0
            while offset < len(binary) - chunk_size:
                list_lines.append(struct.unpack_from(f"={format_string}", binary, offset))
                offset += chunk_size
        data = np.stack(list_lines)
        return data

    def read_timestamps(self, data: np.ndarray) -> np.ndarray:
        time = data[:, self.time_channel]
        return (time - time.min()) / (time.max() - time.min())

    def read_point_cloud(self, data: np.ndarray) -> np.ndarray:
        return data[:, :3]
