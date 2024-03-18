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
from typing import Optional

from pydantic import BaseModel


class DataConfig(BaseModel):
    max_range: float = 100.0
    min_range: float = 5.0
    deskew: bool = False


class MappingConfig(BaseModel):
    voxel_size: Optional[float] = None  # default: take it from data
    max_points_per_voxel: int = 20


class RegistrationConfig(BaseModel):
    max_num_iterations: Optional[int] = 500
    convergence_criterion: Optional[float] = 0.0001
    max_num_threads: Optional[int] = 0  # 0 means automatic


class AdaptiveThresholdConfig(BaseModel):
    fixed_threshold: Optional[float] = None
    initial_threshold: float = 2.0
    min_motion_th: float = 0.1
