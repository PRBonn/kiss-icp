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
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Optional

from easydict import EasyDict
import yaml

# TODO(Nacho)! Make sure that default values are read from this config file when not provided


@dataclass
class KISSConfig:
    out_dir: str = "results"

    @dataclass
    class data:
        preprocess: bool = True
        correct_scan: bool = True
        lidar_frequency: float = 10.0
        max_range: float = 100.0
        min_range: float = 0.0
        deskew: bool = False

    @dataclass
    class mapping:
        voxel_size: float  # default: take it from data
        max_points_per_voxel: int = 20

    @dataclass
    class adaptive_threshold:
        fixed_threshold: Optional[float]
        initial_threshold: float = 1.0
        min_motion_th: float = 0.01

    @staticmethod
    def from_dict(config: Dict) -> KISSConfig:
        return EasyDict(config)


def load_config(path) -> KISSConfig:
    try:
        return EasyDict(yaml.safe_load(open(path)))
    except FileNotFoundError as err:
        raise FileNotFoundError(f"{path} file doesn't exist") from err


def write_config(config: KISSConfig, filename: str):
    with open(filename, "w") as outfile:
        yaml.dump(config.__dict__, outfile, default_flow_style=False)
