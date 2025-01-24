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
import os
import sys
from pathlib import Path

import numpy as np


class NCLTDataset:
    """Adapted from PyLidar-SLAM"""

    def __init__(self, data_dir: Path, *_, **__):
        self.sequence_id = os.path.basename(data_dir)
        self.sequence_dir = os.path.join(os.path.realpath(data_dir), "")
        self.scans_dir = os.path.join(self.sequence_dir, "velodyne_sync")
        scan_files = np.array(sorted(os.listdir(str(self.scans_dir))), dtype=str)
        poses_file = os.path.realpath(
            os.path.join(
                self.sequence_dir,
                "..",
                f"ground_truth/groundtruth_{self.sequence_id}.csv",
            )
        )
        gt_data = np.loadtxt(poses_file, delimiter=",")
        self.timestamps, timestamp_filter = self.load_valid_timestamps(gt_data, scan_files)
        self.scan_files = scan_files[timestamp_filter]
        self.gt_poses = self.load_gt_poses(gt_data)

        # Visualization Options
        self.use_global_visualizer = True

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(os.path.join(self.scans_dir, self.scan_files[idx]))

    def read_point_cloud(self, file_path: str):
        def _convert(x_s, y_s, z_s):
            # Copied from http://robots.engin.umich.edu/nclt/python/read_vel_sync.py
            scaling = 0.005
            offset = -100.0

            x = x_s * scaling + offset
            y = y_s * scaling + offset
            z = z_s * scaling + offset
            return x, y, z

        binary = np.fromfile(file_path, dtype=np.int16)
        x = np.ascontiguousarray(binary[::4])
        y = np.ascontiguousarray(binary[1::4])
        z = np.ascontiguousarray(binary[2::4])
        x = x.astype(np.float32).reshape(-1, 1)
        y = y.astype(np.float32).reshape(-1, 1)
        z = z.astype(np.float32).reshape(-1, 1)
        x, y, z = _convert(x, y, z)
        # Flip to have z pointing up
        points = np.concatenate([x, -y, -z], axis=1)
        return points.astype(np.float64)

    @staticmethod
    def load_valid_timestamps(gt_data: np.ndarray, scan_files: np.ndarray):
        # Ground truth timestamps and LiDARs don't match, interpolate
        gt_t = gt_data[:, 0]
        # Limit the sequence to timestamps for which a ground truth exists
        timestamps = np.array(
            [os.path.basename(file).split(".")[0] for file in scan_files], dtype=np.int64
        )
        filter_ = (timestamps > np.min(gt_t)) * (timestamps < np.max(gt_t))
        return timestamps[filter_], filter_

    def load_gt_poses(self, gt_data: np.ndarray):
        try:
            from scipy import interpolate
            from scipy.spatial.transform import Rotation
        except ImportError:
            print('NCLT dataloader requires scipy: "pip install scipy"')
            sys.exit(1)

        inter = interpolate.interp1d(gt_data[:, 0], gt_data[:, 1:], kind="nearest", axis=0)

        # Limit the sequence to timestamps for which a ground truth exists
        gt = inter(self.timestamps)
        gt_tr = gt[:, :3]
        gt_euler = gt[:, 3:][:, [2, 1, 0]]
        gt_rot = Rotation.from_euler("ZYX", gt_euler).as_matrix()

        gt = np.eye(4, dtype=np.float32).reshape(1, 4, 4).repeat(gt.shape[0], axis=0)
        gt[:, :3, :3] = gt_rot
        gt[:, :3, 3] = gt_tr

        gt = np.einsum(
            "nij,jk->nik",
            gt,
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=np.float32,
            ),
        )
        gt = np.einsum(
            "ij,njk->nik",
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=np.float32,
            ),
            gt,
        )

        return gt
