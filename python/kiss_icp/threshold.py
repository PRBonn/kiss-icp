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

from kiss_icp.config import KISSConfig
from kiss_icp.pybind import kiss_icp_pybind


def get_threshold_estimator(config: KISSConfig):
    if config.adaptive_threshold.fixed_threshold is not None:
        return FixedThreshold(config.adaptive_threshold.fixed_threshold)
    return AdaptiveThreshold(config)


class FixedThreshold:
    def __init__(self, fixed_threshold: float):
        self.fixed_threshold = fixed_threshold

    def get_threshold(self):
        return self.fixed_threshold

    def update_model_deviation(self, model_deviation):
        pass


class AdaptiveThreshold:
    def __init__(self, config: KISSConfig):
        self._estimator = kiss_icp_pybind._AdaptiveThreshold(
            initial_threshold=config.adaptive_threshold.initial_threshold,
            min_motion_th=config.adaptive_threshold.min_motion_th,
            max_range=config.data.max_range,
        )

    def get_threshold(self):
        return self._estimator._compute_threshold()

    def update_model_deviation(self, model_deviation: np.ndarray):
        self._estimator._update_model_deviation(model_deviation=model_deviation)
