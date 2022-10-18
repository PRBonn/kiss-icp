# KISS-ICP: In Defense of Point-to-Point ICP – Simple, Accurate, and Robust Registration If Done the Right Way

![Ubuntu](https://img.shields.io/badge/Ubuntu-333333?style=flat&logo=ubuntu)
![Windows](https://img.shields.io/badge/Windows-333333?style=flat&logo=windows&logoColor=blue)
![macOS](https://img.shields.io/badge/-macOS-333333?style=flat&logo=apple)

![overview](https://user-images.githubusercontent.com/21349875/190433899-a917d7a4-23b1-4247-8291-ae000e9e7871.png)

_Point cloud maps (blue) generated online by our proposed odometry pipeline on different datasets with the same set of parameters.
We depict the latest scan in yellow. The scans are recorded using different sensors with different point densities, different orientations,
and different shooting patterns. The automotive example stems from the MulRan dataset. The drone of the Voxgraph dataset
and the segway robot used in the NCLT dataset show a high acceleration motion profile. The handheld mechanical LiDAR of LOAM
Livox has a completely different shooting pattern than the commonly used rotating mechanical LiDAR._

## 📰 NEWS!!! 📰: Preprint version of the paper now available on [arXiv](https://arxiv.org/pdf/2209.15397.pdf)

Please keep in mind that this publication is under review and might change in the future, stay tnned for the final version.

## 📰 NEWS!!! 📰: BETA-release (only for testing the pipeline)

We released a python-package supported on **macOS**, **Windows**, and **Linux** to test the KISS-ICP pipeline. The entire source code will be available after the review process of the paper is finished.

To get started, just run

```sh
pip install kiss-icp
```

If you also want to use the *(optional)* visualizer, you need to install Open3D or

```sh
pip install "kiss-icp[all]"
```

Next, follow the instructions on how to run the system by typing:

```sh
kiss_icp_pipeline --help
```

This should print the following help message:
![out](https://user-images.githubusercontent.com/21349875/193282970-25a400aa-ebcd-487a-b839-faa04eeca5b9.png)


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
