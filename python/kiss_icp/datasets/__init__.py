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
from pathlib import Path
from typing import Dict, List


def supported_file_extensions():
    return [
        "bin",
        "pcd",
        "ply",
        "xyz",
        "obj",
        "ctm",
        "off",
        "stl",
    ]


def sequence_dataloaders():
    # TODO: automatically infer this
    return ["kitti", "kitti_raw", "nuscenes"]


def available_dataloaders() -> List:
    import os.path
    import pkgutil

    pkgpath = os.path.dirname(__file__)
    return [name for _, name, _ in pkgutil.iter_modules([pkgpath])]


def jumpable_dataloaders():
    _jumpable_dataloaders = available_dataloaders()
    _jumpable_dataloaders.remove("mcap")
    _jumpable_dataloaders.remove("ouster")
    _jumpable_dataloaders.remove("rosbag")
    return _jumpable_dataloaders


def dataloader_types() -> Dict:
    import ast
    import importlib

    dataloaders = available_dataloaders()
    _types = {}
    for dataloader in dataloaders:
        script = importlib.util.find_spec(f".{dataloader}", __name__).origin
        with open(script) as f:
            tree = ast.parse(f.read(), script)
            classes = [cls for cls in tree.body if isinstance(cls, ast.ClassDef)]
            _types[dataloader] = classes[0].name  # assuming there is only 1 class
    return _types


def dataset_factory(dataloader: str, data_dir: Path, *args, **kwargs):
    import importlib

    dataloader_type = dataloader_types()[dataloader]
    module = importlib.import_module(f".{dataloader}", __name__)
    assert hasattr(module, dataloader_type), f"{dataloader_type} is not defined in {module}"
    dataset = getattr(module, dataloader_type)
    return dataset(data_dir=data_dir, *args, **kwargs)
