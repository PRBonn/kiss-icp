# KISS-ICP Evaluation

In this folder you can find a few Python notebooks to run a complete evaluation of KISS-ICP without
much effort.

## Datasets evaluated

**NOTE:** In this repository we add the empty notebooks to use it for running experiments on your
local machine. If you want to see the results of these experiments you can click on any click below
or directly access the already runned notebooks [here](https://nbviewer.org/github/nachovizzo/kiss-icp/tree/main/evaluation/)

- [KITTI Odometry](https://nbviewer.org/github/nachovizzo/kiss-icp/blob/main/evaluation/kitti.ipynb)
- [KITTI RAW Odometry (no motion compensation applied)](https://nbviewer.org/github/nachovizzo/kiss-icp/blob/main/evaluation/kitti_raw.ipynb)
- [Newer College Dataset](https://nbviewer.org/github/nachovizzo/kiss-icp/blob/main/evaluation/newer_college.ipynb)
- [MulRan Dataset](https://nbviewer.org/github/nachovizzo/kiss-icp/blob/main/evaluation/mulran.ipynb)

## Where is the NCLT dataset?

We discourage using NCLT to evaluate odometry systems: misalignments in the ground truth poses,
missing frames, and inconsistencies in the data make the evaluation of odometry systems on such a
dataset not a good evaluation tool from our perspective. However, we provide the results here for
the sake of completeness in the manuscript.

## What about all the other datasets?

Here we just want to add an easy evaluation bench for the KISS-ICP pipeline. Much more datasets are
currently supported by our system and more are to come, an in-complete list of such datasets
includes:

- apollo
- boreas
- kitti
- kitti_raw
- mulran
- ncd
- nclt
- nuscenes
- paris_luco
- rosbag
- rosbag2
- generic folders containing files with extensions:
  - bin
  - pcd
  - ply
  - xyz
  - obj
  - ctm
  - off
  - st

## What about all the other experiments from the paper?

The experiment missing from this evaluation set are those whe require to modify the C++ core
library, therefore we could not add Python notebooks for those experiments, but should be easy to
run them once the full source code is available.
