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
from pathlib import Path
import sys


class RosbagDataset:
    def __init__(self, data_dir: Path, topic: str, *_, **__):
        try:
            import rosbag
        except ModuleNotFoundError:
            print('python rosbag is not installed, run "sudo apt install python3-rosbag"')
            sys.exit(1)

        from kiss_icp.tools.point_cloud2 import read_point_cloud

        self.read_point_cloud = read_point_cloud
        self.sequence_id = os.path.basename(data_dir).split(".")[0]

        # bagfile
        self.bagfile = data_dir
        self.bag = rosbag.Bag(data_dir, mode="r")
        self.topic = self.check_topic(topic)

        # Get an iterable
        self.n_scans = self.bag.get_message_count(topic_filters=self.topic)
        self.msgs = self.bag.read_messages(topics=[self.topic])

        # Visualization Options
        self.use_global_visualizer = True

    def __len__(self):
        return self.n_scans

    def __getitem__(self, idx):
        # TODO: implemnt [idx], expose field_names
        _, msg, _ = next(self.msgs)
        return self.read_point_cloud(msg)

    def check_topic(self, topic: str) -> str:
        # when user specified the topic don't check
        if topic:
            return topic

        # Extract all PointCloud2 msg topics from the bagfile
        point_cloud_topics = [
            topic
            for topic in self.bag.get_type_and_topic_info().topics.items()
            if topic[1].msg_type == "sensor_msgs/PointCloud2"
        ]

        if len(point_cloud_topics) == 1:
            # this is the string topic name, go figure out
            return point_cloud_topics[0][0]

        # In any other case we consider this an error
        if len(point_cloud_topics) == 0:
            print("[ERROR] Your bagfile does not contain any sensor_msgs/PointCloud2 topic")
        if len(point_cloud_topics) > 1:
            print("Multiple sensor_msgs/PointCloud2 topics available.")
            print("Please provide one of the following topics with the --topic flag")
            for topic_tuple in point_cloud_topics:
                print(50 * "-")
                print(f"Topic   {topic_tuple[0]}")
                print(f"\tType      {topic_tuple[1].msg_type}")
                print(f"\tMessages  {topic_tuple[1].message_count}")
                print(f"\tFrequency {topic_tuple[1].frequency}")
            print(50 * "-")
        sys.exit(1)
