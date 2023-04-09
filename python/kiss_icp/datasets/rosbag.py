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
import os
import sys
from pathlib import Path
from typing import Sequence

import natsort


class RosbagDataset:
    def __init__(self, data_dir: Sequence[Path], topic: str, *_, **__):
        """ROS1 / ROS2 bagfile dataloader.

        It can take either one ROS2 bag file or one or more ROS1 bag files belonging to a split bag.
        The reader will replay ROS1 split bags in correct timestamp order.

        TODO: Merge mcap and rosbag dataloaders into 1
        """
        try:
            from rosbags.highlevel import AnyReader
        except ModuleNotFoundError:
            print('rosbags library not installed, run "pip install -U rosbags"')
            sys.exit(1)

        from kiss_icp.tools.point_cloud2 import read_point_cloud

        self.read_point_cloud = read_point_cloud

        # FIXME: This is quite hacky, trying to guess if we have multiple .bag, one or a dir
        if isinstance(data_dir, Path):
            self.sequence_id = os.path.basename(data_dir).split(".")[0]
            self.bag = AnyReader([data_dir])
        else:
            self.sequence_id = os.path.basename(data_dir[0]).split(".")[0]
            self.bag = AnyReader(data_dir)
            print("Reading multiple .bag files in directory:")
            print("\n".join(natsort.natsorted([path.name for path in self.bag.paths])))
        self.bag.open()
        self.topic = self.check_topic(topic)
        self.n_scans = self.bag.topics[self.topic].msgcount

        # limit connections to selected topic
        connections = [x for x in self.bag.connections if x.topic == self.topic]
        self.msgs = self.bag.messages(connections=connections)
        self.timestamps = []

        # Visualization Options
        self.use_global_visualizer = True

    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        connection, timestamp, rawdata = next(self.msgs)
        self.timestamps.append(self.to_sec(timestamp))
        msg = self.bag.deserialize(rawdata, connection.msgtype)
        return self.read_point_cloud(msg)

    @staticmethod
    def to_sec(nsec: int):
        return float(nsec) / 1e9

    def get_frames_timestamps(self) -> list:
        return self.timestamps

    def check_topic(self, topic: str) -> str:
        # Extract all PointCloud2 msg topics from the bagfile
        point_cloud_topics = [
            topic[0]
            for topic in self.bag.topics.items()
            if topic[1].msgtype == "sensor_msgs/msg/PointCloud2"
        ]

        def print_available_topics_and_exit():
            print(50 * "-")
            for t in point_cloud_topics:
                print(f"--topic {t}")
            print(50 * "-")
            sys.exit(1)

        if topic and topic in point_cloud_topics:
            return topic
        # when user specified the topic check that exists
        if topic and topic not in point_cloud_topics:
            print(
                f'[ERROR] Dataset does not containg any msg with the topic name "{topic}". '
                "Please select one of the following topics with the --topic flag"
            )
            print_available_topics_and_exit()
        if len(point_cloud_topics) > 1:
            print(
                "Multiple sensor_msgs/msg/PointCloud2 topics available."
                "Please select one of the following topics with the --topic flag"
            )
            print_available_topics_and_exit()

        if len(point_cloud_topics) == 0:
            print("[ERROR] Your dataset does not contain any sensor_msgs/msg/PointCloud2 topic")
        if len(point_cloud_topics) == 1:
            return point_cloud_topics[0]
