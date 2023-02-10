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
from pathlib import Path
from typing import Any, Dict, Optional

from pydantic import BaseModel, BaseSettings, PrivateAttr
from pydantic.env_settings import InitSettingsSource
import yaml


class DataConfig(BaseModel):
    preprocess: bool = True
    correct_scan: bool = True
    frame_rate: float = 10.0
    max_range: float = 100.0
    min_range: float = 0.0
    deskew: bool = False


class MappingConfig(BaseModel):
    voxel_size: Optional[float] = None  # default: take it from data
    max_points_per_voxel: int = 20


class AdaptiveThresholdConfig(BaseModel):
    fixed_threshold: Optional[float] = None
    initial_threshold: float = 2.0
    min_motion_th: float = 0.1


class KISSConfig(BaseSettings):
    out_dir: str = "results"
    data: DataConfig = DataConfig()
    mapping: MappingConfig = MappingConfig()
    adaptive_threshold: AdaptiveThresholdConfig = AdaptiveThresholdConfig()

    _config_file: Optional[Path] = PrivateAttr()

    def __init__(self, config_file: Optional[Path] = None, *args, **kwargs):
        """
        Initialize a new configuration.

        :param config_file: optional path to a YAML configuration file.
            The file can selectively specify values to override default values.
        """
        self._config_file = config_file
        super().__init__(*args, **kwargs)

    def _yaml_source(self) -> Dict[str, Any]:
        """Function that implements the `pydantic.env_settings.SettingsSourceCallable` interface."""
        data = None
        if self._config_file is not None:
            with open(self._config_file) as f:
                data = yaml.safe_load(f)
        return data or {}

    class Config:
        # Register our custom yaml loader
        # See https://docs.pydantic.dev/usage/settings/#adding-sources
        @classmethod
        def customise_sources(
            cls,
            init_settings,
            env_settings,
            file_secret_settings,
        ):
            return init_settings, KISSConfig._yaml_source


def load_config(
    config_file: Optional[Path], deskew: Optional[bool], max_range: Optional[float]
) -> KISSConfig:
    config = KISSConfig(config_file=config_file)

    # Override defaults from command line
    if deskew is not None:
        config.data.deskew = deskew
    if max_range is not None:
        config.data.max_range = max_range

    # Check if there is a possible mistake
    if config.data.max_range < config.data.min_range:
        print("[WARNING] max_range is smaller than min_range, settng min_range to 0.0")
        config.data.min_range = 0.0

    # Use specified voxel size or compute one using the max range
    if config.mapping.voxel_size is None:
        config.mapping.voxel_size = float(config.data.max_range / 100.0)

    return config


def write_config(config: KISSConfig, filename: str):
    with open(filename, "w") as outfile:
        yaml.dump(config.dict(), outfile, default_flow_style=False)
