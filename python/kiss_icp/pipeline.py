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
import contextlib
import datetime
import os
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
from pyquaternion import Quaternion

from kiss_icp.config import load_config, write_config
from kiss_icp.kiss_icp import KissICP
from kiss_icp.metrics import absolute_trajectory_error, sequence_error
from kiss_icp.tools.pipeline_results import PipelineResults
from kiss_icp.tools.progress_bar import get_progress_bar
from kiss_icp.tools.visualizer import RegistrationVisualizer, StubVisualizer


class OdometryPipeline:
    def __init__(
        self,
        dataset,
        config: Optional[Path] = None,
        deskew: Optional[bool] = False,
        max_range: Optional[float] = None,
        visualize: bool = False,
        n_scans: int = -1,
        jump: int = 0,
    ):
        self._dataset = dataset
        self._n_scans = (
            len(self._dataset) - jump if n_scans == -1 else min(len(self._dataset) - jump, n_scans)
        )
        self._jump = jump
        self._first = jump
        self._last = self._jump + self._n_scans

        # Config and output dir
        self.config = load_config(config, deskew=deskew, max_range=max_range)
        self.results_dir = None

        # Pipeline
        self.odometry = KissICP(config=self.config)
        self.results = PipelineResults()
        self.times = []
        self.poses = self.odometry.poses
        self.has_gt = hasattr(self._dataset, "gt_poses")
        self.gt_poses = self._dataset.gt_poses[self._first : self._last] if self.has_gt else None
        self.dataset_name = self._dataset.__class__.__name__
        self.dataset_sequence = (
            self._dataset.sequence_id
            if hasattr(self._dataset, "sequence_id")
            else os.path.basename(self._dataset.data_dir)
        )

        # Visualizer
        self.visualizer = RegistrationVisualizer() if visualize else StubVisualizer()
        if hasattr(self._dataset, "use_global_visualizer"):
            self.visualizer.global_view = self._dataset.use_global_visualizer

    # Public interface  ------
    def run(self):
        self._run_pipeline()
        self._run_evaluation()
        self._create_output_dir()
        self._write_result_poses()
        self._write_gt_poses()
        self._write_cfg()
        self._write_log()
        return self.results

    # Private interface  ------
    def _run_pipeline(self):
        for idx in get_progress_bar(self._first, self._last):
            raw_frame, timestamps = self._next(idx)
            start_time = time.perf_counter_ns()
            source, keypoints = self.odometry.register_frame(raw_frame, timestamps)
            self.times.append(time.perf_counter_ns() - start_time)
            self.visualizer.update(source, keypoints, self.odometry.local_map, self.poses[-1])

    def _next(self, idx):
        """TODO: re-arrange this logic"""
        dataframe = self._dataset[idx]
        try:
            frame, timestamps = dataframe
        except ValueError:
            frame = dataframe
            timestamps = np.zeros(frame.shape[0])
        return frame, timestamps

    @staticmethod
    def save_poses_kitti_format(filename: str, poses: List[np.ndarray]):
        def _to_kitti_format(poses: np.ndarray) -> np.ndarray:
            return np.array([np.concatenate((pose[0], pose[1], pose[2])) for pose in poses])

        np.savetxt(fname=f"{filename}_kitti.txt", X=_to_kitti_format(poses))

    @staticmethod
    def save_poses_tum_format(filename, poses, timestamps):
        def _to_tum_format(poses, timestamps):
            tum_data = []
            with contextlib.suppress(ValueError):
                for idx in range(len(poses)):
                    tx, ty, tz = poses[idx][:3, -1].flatten()
                    qw, qx, qy, qz = Quaternion(matrix=poses[idx], atol=0.01).elements
                    tum_data.append([float(timestamps[idx]), tx, ty, tz, qx, qy, qz, qw])
            return np.array(tum_data).astype(np.float64)

        np.savetxt(fname=f"{filename}_tum.txt", X=_to_tum_format(poses, timestamps), fmt="%.4f")

    def _calibrate_poses(self, poses):
        return (
            self._dataset.apply_calibration(poses)
            if hasattr(self._dataset, "apply_calibration")
            else poses
        )

    def _get_frames_timestamps(self):
        return (
            self._dataset.get_frames_timestamps()
            if hasattr(self._dataset, "get_frames_timestamps")
            else np.arange(0, len(self.poses), 1.0)
        )

    def _save_poses(self, filename: str, poses, timestamps):
        np.save(filename, poses)
        self.save_poses_kitti_format(filename, poses)
        self.save_poses_tum_format(filename, poses, timestamps)

    def _write_result_poses(self):
        self._save_poses(
            filename=f"{self.results_dir}/{self.dataset_sequence}_poses",
            poses=self._calibrate_poses(self.poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _write_gt_poses(self):
        if not self.has_gt:
            return
        self._save_poses(
            filename=f"{self.results_dir}/{self._dataset.sequence_id}_gt",
            poses=self._calibrate_poses(self.gt_poses),
            timestamps=self._get_frames_timestamps(),
        )

    def _run_evaluation(self):
        if not self.has_gt:
            print("[WARNING] No GT poses available, skipping evaluation")
            return

        def _get_fps():
            total_time_s = sum(self.times) * 1e-9
            return float(len(self.times) / total_time_s)

        # Run metrics evaluation
        avg_tra, avg_rot = sequence_error(self.gt_poses, self.poses)
        ate_rot, ate_trans = absolute_trajectory_error(self.gt_poses, self.poses)
        avg_fps = int(np.floor(_get_fps()))
        avg_ms = 1e3 * (1 / avg_fps)

        self.results.append(desc="Average Translation Error", units="%", value=avg_tra)
        self.results.append(desc="Average Rotational Error", units="deg/m", value=avg_rot)
        self.results.append(desc="Absoulte Trajectory Error (ATE)", units="m", value=ate_trans)
        self.results.append(desc="Absoulte Rotational Error (ARE)", units="rad", value=ate_rot)
        self.results.append(desc="Average Frequency", units="Hz", value=avg_fps, trunc=True)
        self.results.append(desc="Average Runtime", units="ms", value=avg_ms, trunc=True)

    def _write_log(self):
        if not self.results.empty():
            self.results.log_to_file(
                f"{self.results_dir}/result_metrics.log",
                f"Results for {self.dataset_name} Sequence {self.dataset_sequence}",
            )

    def _write_cfg(self):
        write_config(self.config, os.path.join(self.results_dir, "config.yml"))

    @staticmethod
    def _get_results_dir(out_dir: str):
        def get_current_timestamp() -> str:
            return datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        results_dir = os.path.join(os.path.realpath(out_dir), get_current_timestamp())
        latest_dir = os.path.join(os.path.realpath(out_dir), "latest")
        os.makedirs(results_dir, exist_ok=True)
        os.unlink(latest_dir) if os.path.exists(latest_dir) or os.path.islink(latest_dir) else None
        os.symlink(results_dir, latest_dir)
        return results_dir

    def _create_output_dir(self):
        self.results_dir = self._get_results_dir(self.config.out_dir)
