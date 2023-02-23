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
import os
import shutil

from setuptools import find_packages
from skbuild import setup


@contextlib.contextmanager
def skbuild_isolated_context():
    """This hack should be removed soon: https://github.com/scikit-build/scikit-build/issues/755.

    For whatever reason, when building the package in isolated mode (PEP 517) the cmake and ninja
    dependencies (that are indeed installed by the build system)do not expose the binary paths.
    Internally scikit-build uses $PATH/cmake and $PATH/ninja to analyze if those tools are installed
    on the system or not. Since this is not working, the workaround for those machines that do not
    have a local installation of the tools is to update the path with the given binary dirs from the
    isolation-installed packages.
    """

    def cmd_exists(cmd):
        return shutil.which(cmd) is not None

    if not cmd_exists("cmake"):
        import cmake  # pylint: disable=import-outside-toplevel

        os.environ["PATH"] += os.pathsep + cmake.CMAKE_BIN_DIR

    if not cmd_exists("ninja"):
        import ninja  # pylint: disable=import-outside-toplevel

        os.environ["PATH"] += os.pathsep + ninja.BIN_DIR

    yield


with skbuild_isolated_context():
    setup(
        packages=find_packages(),
        cmake_install_dir="kiss_icp/pybind/",
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
