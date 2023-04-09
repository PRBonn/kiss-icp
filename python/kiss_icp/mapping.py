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
from typing import Tuple

import numpy as np

from kiss_icp.config import KISSConfig
from kiss_icp.pybind import kiss_icp_pybind


def get_voxel_hash_map(config: KISSConfig):
    return VoxelHashMap(
        voxel_size=config.mapping.voxel_size,
        max_distance=config.data.max_range,
        max_points_per_voxel=config.mapping.max_points_per_voxel,
    )


class VoxelHashMap:
    def __init__(self, voxel_size: float, max_distance: float, max_points_per_voxel: int):
        self._internal_map = kiss_icp_pybind._VoxelHashMap(
            voxel_size=voxel_size,
            max_distance=max_distance,
            max_points_per_voxel=max_points_per_voxel,
        )

    def clear(self):
        return self._internal_map._clear()

    def empty(self):
        return self._internal_map._empty()

    def update(self, points: np.ndarray, pose: np.ndarray = np.eye(4)):
        """Add points to the inernal map representaion.

        The origin is needed to remove the far away poitns

        TODO(NACHO): Use similar overload API as we did for VDBFusion
        """
        self._internal_map._update(kiss_icp_pybind._Vector3dVector(points), pose)

    def point_cloud(self) -> np.ndarray:
        """Return the internal representaion as a np.array (pointcloud)."""
        return np.asarray(self._internal_map._point_cloud())

    def get_correspondences(
        self,
        points: np.ndarray,
        max_correspondance_distance: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get the pair of {source, target} pointcloud of the same size."""
        _points = kiss_icp_pybind._Vector3dVector(points)
        source, target = self._internal_map._get_correspondences(
            _points, max_correspondance_distance
        )
        return np.asarray(source), np.asarray(target)
