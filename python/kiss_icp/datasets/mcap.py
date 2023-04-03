# MIT License
#
# Copyright (c) 2023 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
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


class McapDataloader:
    def __init__(self, data_dir: str, topic: str, *_, **__):
        """Standalone .mcap dataloader withouth any ROS distribution."""
        # Conditional imports to avoid injecting dependencies for non mcap users
        try:
            from mcap_ros2.reader import read_ros2_messages
            from mcap.reader import make_reader
        except ImportError as e:
            print("mcap plugins not installed: 'pip install mcap-ros2-support'")
            exit(1)

        from kiss_icp.tools.point_cloud2 import read_point_cloud

        # we expect `data_dir` param to be a path to the .mcap file, so rename for clarity
        assert os.path.isfile(data_dir), "mcap dataloader expects an existing MCAP file"
        mcap_file = str(data_dir)
        self.data_dir = os.path.dirname(data_dir)

        self.bag = make_reader(open(mcap_file, "rb"))
        self.summary = self.bag.get_summary()
        self.topic = self.check_topic(topic)
        self.n_scans = self._get_n_scans()
        self.msgs = read_ros2_messages(mcap_file, topics=topic)
        self.read_point_cloud = read_point_cloud
        self.use_global_visualizer = True

    def __del__(self):
        if hasattr(self, "bag"):
            del self.bag

    def __getitem__(self, idx):
        msg = next(self.msgs).ros_msg
        return self.read_point_cloud(msg)

    def __len__(self):
        return self.n_scans

    def _get_n_scans(self) -> int:
        return sum(
            count
            for (id, count) in self.summary.statistics.channel_message_counts.items()
            if self.summary.channels[id].topic == self.topic
        )

    def check_topic(self, topic: str) -> str:
        # when user specified the topic don't check
        if topic:
            return topic

        # Extract schema id from the .mcap file that encodes the PointCloud2 msg
        schema_id = [
            schema.id
            for schema in self.summary.schemas.values()
            if schema.name == "sensor_msgs/msg/PointCloud2"
        ][0]

        point_cloud_channels = [
            channel for channel in self.summary.channels.values() if channel.schema_id == schema_id
        ]
        if len(point_cloud_channels) == 0:
            print("[ERROR] Your mcap does not contain any sensor_msgs/msg/PointCloud2 topic")
        if len(point_cloud_channels) == 1:
            return point_cloud_channels[0].topic
        if len(point_cloud_channels) > 1:
            print("Multiple sensor_msgs/msg/PointCloud2 topics available.")
            print("Please select one of the following topics with the --topic flag")
            print(50 * "-")
            for channel in point_cloud_channels:
                print(f"--topic {channel.topic}")
            print(50 * "-")
        sys.exit(1)
