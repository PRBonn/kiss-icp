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

import importlib
import os
from pathlib import Path
import sys

import numpy as np


class RosbagDataset:
    def __init__(self, data_dir: Path, topic: str, *_, **__):
        try:
            from rosbags import rosbag2
        except ModuleNotFoundError:
            print('rosbag2 reader is not installed, run "pip install rosbags"')
            sys.exit(1)
        
        from kiss_icp.tools.point_cloud2 import read_point_cloud
        self.read_point_cloud = read_point_cloud

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
        return self.read_point_cloud(msg)

    def check_for_topics(self):
        if self.topic:
            return
        print("Please provide one of the following topics with the --topic flag")
        print("PointCloud2 topics available:")
        for topic, topic_info in self.bag.topics.items():
            if topic_info.msgtype == "sensor_msgs/msg/PointCloud2":
                print(topic)
        sys.exit(1)
