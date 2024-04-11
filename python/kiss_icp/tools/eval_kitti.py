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

import typer
import numpy as np
from pathlib import Path
from kiss_icp.metrics import sequence_error, absolute_trajectory_error

run = typer.Typer(add_completion=False, rich_markup_mode="rich")


def load_poses(path):
    poses = np.loadtxt(path, delimiter=" ")
    n = poses.shape[0]
    poses = np.concatenate(
        (poses, np.zeros((n, 3), dtype=np.float32), np.ones((n, 1), dtype=np.float32)),
        axis=1,
    )
    poses = [poses[i, :].reshape(4, 4) for i in range(n)]
    return poses


@run.command()
def eval(
    poses_path: Path = typer.Argument(
        ...,
        help="Path to the file with poses in KITTI format.",
    ),
    ref_path: Path = typer.Argument(
        ...,
        help="Path to the file with reference poses in KITTI format.",
    ),
):
    poses = load_poses(poses_path)
    gt_poses = load_poses(ref_path)
    avg_tra, avg_rot = sequence_error(gt_poses, poses)
    ate_rot, ate_trans = absolute_trajectory_error(gt_poses, poses)
    print(f"Average Translation Error           {avg_tra:.3f}")
    print(f"Average Rotational Error            {avg_rot:.3f}")
    print(f"Absolute Trajectory Error (ATE)     {ate_trans:.3f}")
    print(f"Absolute Rotational Error (ARE)     {ate_rot:.3f}")


if __name__ == "__main__":
    run()
