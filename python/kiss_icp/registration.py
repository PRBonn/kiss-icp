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
import numpy as np

from kiss_icp.config.parser import KISSConfig
from kiss_icp.mapping import VoxelHashMap
from kiss_icp.pybind import kiss_icp_pybind


def get_registration(config: KISSConfig):
    return Registration(
        max_num_iterations=config.registration.max_num_iterations,
        convergence_criterion=config.registration.convergence_criterion,
        max_num_threads=config.registration.max_num_threads,
    )


class Registration:
    def __init__(
        self,
        max_num_iterations: int,
        convergence_criterion: float,
        max_num_threads: int = 0,
    ):
        self._registration = kiss_icp_pybind._Registration(
            max_num_iterations=max_num_iterations,
            convergence_criterion=convergence_criterion,
            max_num_threads=max_num_threads,
        )

    def align_points_to_map(
        self,
        points: np.ndarray,
        voxel_map: VoxelHashMap,
        initial_guess: np.ndarray,
        max_correspondance_distance: float,
        kernel: float,
    ) -> np.ndarray:
        return self._registration._align_points_to_map(
            points=kiss_icp_pybind._Vector3dVector(points),
            voxel_map=voxel_map._internal_map,
            initial_guess=initial_guess,
            max_correspondance_distance=max_correspondance_distance,
            kernel=kernel,
        )
