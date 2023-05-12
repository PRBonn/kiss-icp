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
from setuptools import find_packages
from skbuild import setup

setup(
    packages=find_packages(),
    cmake_install_dir="kiss_icp/pybind/",
    cmake_install_target="install_python_bindings",
    entry_points={"console_scripts": ["kiss_icp_pipeline=kiss_icp.tools.cmd:run"]},
    install_requires=[
        "natsort",
        "numpy",
        "plyfile",
        "pydantic",
        "pyquaternion",
        "rich",
        "tqdm",
        "typer[all]>=0.6.0",
    ],
    extras_require={
        "visualizer": [
            "open3d>=0.13",
        ],
        "all": [
            "PyYAML",
            "open3d>=0.13",
            "ouster-sdk>=0.7.1",
            "pyntcloud",
            "trimesh",
        ],
    },
)
