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

import natsort
import numpy as np

from kiss_icp.datasets import supported_file_extensions


class GenericDataset:
    def __init__(self, data_dir: Path, *_, **__):
        # Config stuff
        self.sequence_id = os.path.basename(os.path.abspath(data_dir))
        self.scans_dir = os.path.join(os.path.realpath(data_dir), "")
        self.scan_files = np.array(
            natsort.natsorted(
                [
                    os.path.join(self.scans_dir, fn)
                    for fn in os.listdir(self.scans_dir)
                    if any(fn.endswith(ext) for ext in supported_file_extensions())
                ]
            ),
            dtype=str,
        )
        if len(self.scan_files) == 0:
            raise ValueError(f"Tried to read point cloud files in {self.scans_dir} but none found")
        self.file_extension = self.scan_files[0].split(".")[-1]
        if self.file_extension not in supported_file_extensions():
            raise ValueError(f"Supported formats are: {supported_file_extensions()}")

        # Obtain the pointcloud reader for the given data folder
        self._read_point_cloud = self._get_point_cloud_reader()

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(self.scan_files[idx])

    def read_point_cloud(self, file_path: str):
        points, timestamps = self._read_point_cloud(file_path)
        return points.astype(np.float64), timestamps.astype(np.float64)

    def _get_point_cloud_reader(self):
        """Attempt to guess with try/catch blocks which is the best point cloud reader to use for
        the given dataset folder. Supported readers so far are:
            - np.fromfile
            - trimesh.load
            - PyntCloud
            - open3d[optional]
        """
        # This is easy, the old KITTI format
        if self.file_extension == "bin":
            print("[WARNING] Reading .bin files, the only format supported is the KITTI format")

            class ReadKITTI:
                def __call__(self, file):
                    return np.fromfile(file, dtype=np.float32).reshape((-1, 4))[:, :3], np.array([])

            return ReadKITTI()

        print('Trying to guess how to read your data: `pip install "kiss-icp[all]"` is required')
        first_scan_file = self.scan_files[0]
        # first try open3d
        try:
            import open3d as o3d

            try_pcd = o3d.t.io.read_point_cloud(first_scan_file)
            if try_pcd.is_empty():
                # open3d binding does not raise an exception if file is unreadable or extension is not supported
                raise Exception("Generic Dataloader| Open3d PointCloud file is empty")

            stamps_keys = ["t", "timestamp", "timestamps", "time", "stamps"]
            stamp_field = None
            for key in stamps_keys:
                try:
                    try_pcd.point[key]
                    stamp_field = key
                    print("Generic Dataloader| found timestamps")
                    break
                except:
                    continue

            class ReadOpen3d:
                def __init__(self, time_field):
                    self.time_field = time_field
                    if self.time_field is None:
                        self.get_timestamps = lambda _: np.array([])
                    else:
                        self.get_timestamps = lambda pcd: pcd.point[self.time_field].numpy().ravel()

                def __call__(self, file):
                    pcd = o3d.t.io.read_point_cloud(file)
                    points = pcd.point.positions.numpy()
                    return points, self.get_timestamps(pcd)

            return ReadOpen3d(stamp_field)
        except:
            pass

        try:
            import trimesh

            trimesh.load(first_scan_file)

            class ReadTriMesh:
                def __call__(self, file):
                    return np.asarray(trimesh.load(file).vertices), np.array([])

            return ReadTriMesh()
        except:
            pass

        try:
            from pyntcloud import PyntCloud

            PyntCloud.from_file(first_scan_file)

            class ReadPynt:
                def __call__(self, file):
                    return PyntCloud.from_file(file).points[["x", "y", "z"]].to_numpy(), np.array(
                        []
                    )

            return ReadPynt()
        except:
            print("[ERROR], File format not supported")
            sys.exit(1)
