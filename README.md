# KISS-ICP: In Defense of Point-to-Point ICP – Simple, Accurate, and Robust Registration If Done the Right Way

![ubuntu](https://img.shields.io/badge/ubuntu-333333?style=flat&logo=ubuntu)
![windows](https://img.shields.io/badge/windows-333333?style=flat&logo=windows&logocolor=blue)
![macos](https://img.shields.io/badge/-macos-333333?style=flat&logo=apple)

![overview](https://user-images.githubusercontent.com/21349875/190433899-a917d7a4-23b1-4247-8291-ae000e9e7871.png)

_Point cloud maps (blue) generated online by our proposed odometry pipeline on different datasets with the same set of parameters.
We depict the latest scan in yellow. The scans are recorded using different sensors with different point densities, different orientations,
and different shooting patterns. The automotive example stems from the MulRan dataset. The drone of the Voxgraph dataset
and the segway robot used in the NCLT dataset show a high acceleration motion profile. The handheld mechanical LiDAR of LOAM
Livox has a completely different shooting pattern than the commonly used rotating mechanical LiDAR._

## Install

We released a python-package supported on
![macos](https://img.shields.io/badge/-macos-333333?style=flat&logo=apple),
![windows](https://img.shields.io/badge/windows-333333?style=flat&logo=windows&logocolor=blue), and
![ubuntu](https://img.shields.io/badge/ubuntu-333333?style=flat&logo=ubuntu).


To get started, just run

```sh
pip install kiss-icp
```

If you also want to install all the *(optional)* dependencies, like Open3D for running the visualizer:

```sh
pip install "kiss-icp[all]"
```

Next, follow the instructions on how to run the system by typing:

```sh
kiss_icp_pipeline --help
```

This should print the following help message:
![out](https://user-images.githubusercontent.com/21349875/193282970-25a400aa-ebcd-487a-b839-faa04eeca5b9.png)


## Install (developer mode)

If you plan to modify the code then you need to setup the dev dependencies, luckilly, the only real
requirements are a modern C++ compiler and the `pip` package manager, nothing else!, in Ubuntu-based
sytems this can be done with:

```sh
sudo apt install g++ python3-pip
```

After that you can clone the code and install the python api:
```sh
git clone https://github.com/PRBonn/kiss-icp.git
cd kiss-icp
pip install --verbose .
```

## Install (expert mode)

If you want to have more controll over the build, you should then install `cmake`, ,`ninja`, `tbb`,
`Eigen`, and `pybind11` as extra dependencies in your system, the ubuntu-way of doing this is:

```sh
sudo apt install build-essential libeigen3-dev libtbb-dev pybind11-dev ninja-build
```


## Teaser Video 

https://user-images.githubusercontent.com/38326482/189950820-030fd9e4-406b-4d14-8171-43b134344223.mp4


## Authors

- Ignacio Vizzo 
- Tiziano Guadagnino 
- Benedikt Mersch 
- Louis Wiesmann 
- Jens Behley 
- Cyrill Stachniss

## Citation

If you use this library for any academic work, please cite our original [paper](https://arxiv.org/pdf/2209.15397.pdf).

```bibtex
@article{vizzo2022arxiv,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {arXiv preprint},
  eprint    = {2209.15397v1},
  doi       = {10.48550/ARXIV.2209.15397},
  url       = {https://arxiv.org/pdf/2209.15397.pdf},
  year      = {2022},
  codeurl   = {https://github.com/PRBonn/kiss-icp},
}
```
