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
"""
NOTE: Original implementation of the read_points method taken from  sensor_msgs.point_cloud2.py, all
rights reserver to the original author: Tim Field
"""
from dataclasses import dataclass
import importlib
import math
import os
from pathlib import Path
import struct
import sys
from typing import ClassVar

import numpy as np


class RosbagDataset:
    def __init__(self, data_dir: Path, topic: str, *_, **__):
        try:
            from rosbags import rosbag2
        except ModuleNotFoundError:
            print('rosbag2 reader is not installed, run "pip install rosbags"')
            sys.exit(1)

        self.deserialize_cdr = importlib.import_module("rosbags.serde").deserialize_cdr

        # Config stuff
        self.sequence_id = os.path.basename(data_dir).split(".")[0]

        # bagfile
        self.bagfile = data_dir
        self.bag = rosbag2.Reader(self.bagfile)
        self.bag.open()
        self.topic = topic
        self.check_for_topics()
        self.n_scans = self.bag.topics[self.topic].msgcount

        # limit connections to selected topic
        connections = [x for x in self.bag.connections if x.topic == topic]
        self.msgs = self.bag.messages(connections=connections)

        # Visualization Options
        self.use_global_visualizer = True

    def __del__(self):
        self.bag.close()

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        connection, _, rawdata = next(self.msgs)
        msg = self.deserialize_cdr(rawdata, connection.msgtype)
        points = np.array(list(read_points(msg, field_names=["x", "y", "z"])))

        t_field = None
        for field in msg.fields:
            if field.name == "t" or field.name == "timestamp":
                t_field = field.name
        timestamps = np.ones(points.shape[0])
        if t_field:
            timestamps = np.array(list(read_points(msg, field_names=t_field)))
            timestamps = timestamps / np.max(timestamps)
        return points.astype(np.float64), timestamps

    def check_for_topics(self):
        if self.topic:
            return
        print("Please provide one of the following topics with the --topic flag")
        print("PointCloud2 topics available:")
        for topic, topic_info in self.bag.topics.items():
            if topic_info.msgtype == "sensor_msgs/msg/PointCloud2":
                print(topic)
        sys.exit(1)


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    @dataclass
    class PointField:
        """Class for sensor_msgs/msg/PointField."""

        name: str
        offset: int
        datatype: int
        count: int
        INT8: ClassVar[int] = 1
        UINT8: ClassVar[int] = 2
        INT16: ClassVar[int] = 3
        UINT16: ClassVar[int] = 4
        INT32: ClassVar[int] = 5
        UINT32: ClassVar[int] = 6
        FLOAT32: ClassVar[int] = 7
        FLOAT64: ClassVar[int] = 8
        __msgtype__: ClassVar[str] = "sensor_msgs/msg/PointField"

    _datatypes = {
        PointField.INT8: ("b", 1),
        PointField.UINT8: ("B", 1),
        PointField.INT16: ("h", 2),
        PointField.UINT16: ("H", 2),
        PointField.INT32: ("i", 4),
        PointField.UINT32: ("I", 4),
        PointField.FLOAT32: ("f", 4),
        PointField.FLOAT64: ("d", 8),
    }

    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _datatypes:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _datatypes[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step
