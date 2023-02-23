# KISS-ICP

[![KISS-ICP demo](https://user-images.githubusercontent.com/21349875/211829074-474bec08-0129-4e34-85e7-62265e44a7de.png)](https://user-images.githubusercontent.com/21349875/219626075-d67e9165-31a2-4a1b-8c26-9f04e7d195ec.mp4
)

## Install Python API

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

### Install Python API (developer mode)

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

### Install Python API (expert mode)

If you want to have more controll over the build, you should then install `cmake`, ,`ninja`, `tbb`,
`Eigen`, and `pybind11` as extra dependencies in your system, the ubuntu-way of doing this is:

```sh
sudo apt install build-essential libeigen3-dev libtbb-dev pybind11-dev ninja-build
```

## Citation

If you use this library for any academic work, please cite our original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf).

```bibtex
@article{vizzo2023ral,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {IEEE Robotics and Automation Letters (RA-L)},
  pages     = {1-8},
  doi       = {10.1109/LRA.2023.3236571},
  volume    = {8},
  number    = {2},
  year      = {2023},
  codeurl   = {https://github.com/PRBonn/kiss-icp},
}
```

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=PRBonn/kiss-icp&type=Date)](https://star-history.com/#PRBonn/kiss-icp&Date)
