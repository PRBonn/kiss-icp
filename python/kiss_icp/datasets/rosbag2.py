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


class RosbagDataset:
    def __init__(self, data_dir: Path, topic: str, *_, **__):
        try:
            from rosbags import rosbag2
        except ImportError:
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
        self.topic = self.check_topic(topic)
        self.n_scans = self.bag.topics[self.topic].msgcount

        # limit connections to selected topic
        connections = [x for x in self.bag.connections if x.topic == self.topic]
        self.msgs = self.bag.messages(connections=connections)

        # Visualization Options
        self.use_global_visualizer = True

    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        connection, _, rawdata = next(self.msgs)
        msg = self.deserialize_cdr(rawdata, connection.msgtype)
        return self.read_point_cloud(msg)

    def check_topic(self, topic: str) -> str:
        # when user specified the topic don't check
        if topic:
            return topic

        # Extract all PointCloud2 msg topics from the bagfile
        point_cloud_topics = [
            topic
            for topic in self.bag.topics.items()
            if topic[1].msgtype == "sensor_msgs/msg/PointCloud2"
        ]

        if len(point_cloud_topics) == 1:
            # this is the string topic name, go figure out
            return point_cloud_topics[0][0]

        # In any other case we consider this an error
        if len(point_cloud_topics) == 0:
            print("[ERROR] Your bagfile does not contain any sensor_msgs/PointCloud2 topic")
        if len(point_cloud_topics) > 1:
            print("Multiple sensor_msgs/msg/PointCloud2 topics available.")
            print("Please provide one of the following topics with the --topic flag")
            for topic_tuple in point_cloud_topics:
                print(50 * "-")
                print(f"Topic {topic_tuple[0]}")
                print(f"\tType      {topic_tuple[1].msgtype}")
                print(f"\tMessages  {topic_tuple[1].count}")
            print(50 * "-")
        sys.exit(1)
