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
import glob
import os
import re
import sys

import numpy as np


class McapDataloader:
    def __init__(self, data_dir: str, topic: str, *_, **__):
        """Standalone .mcap dataloader without any ROS distribution.

        Accepts either a single .mcap file or a directory containing multiple
        .mcap files. When a directory is given, all .mcap files are read in
        chronological order determined by the first message timestamp on the
        requested topic. If timestamps cannot be read from a file (e.g. the
        topic is absent), the loader falls back to natural filename order
        (i.e. run_1, run_2, ..., run_10 rather than lexicographic order).
        """
        # Conditional imports to avoid injecting dependencies for non mcap users
        try:
            from mcap.reader import make_reader
            from mcap_ros2.reader import read_ros2_messages
        except ImportError:
            print("mcap plugins not installed: 'pip install mcap-ros2-support'")
            exit(1)

        from kiss_icp.tools.point_cloud2 import read_point_cloud

        self.make_reader = make_reader
        self.read_ros2_messages = read_ros2_messages
        self.read_point_cloud = read_point_cloud

        if os.path.isdir(data_dir):
            self.mcap_files = self._collect_files(data_dir, topic)
            self.sequence_id = os.path.basename(str(data_dir).rstrip("/"))
        elif os.path.isfile(data_dir):
            self.mcap_files = [str(data_dir)]
            self.sequence_id = os.path.basename(str(data_dir)).split(".")[0]
        else:
            raise FileNotFoundError(
                f"mcap dataloader expects an existing .mcap file or a directory, got: {data_dir}"
            )

        # Open the first file to validate the topic and build the summary used
        # by check_topic(). For multi-file datasets the summary of the first
        # file is sufficient because all files share the same schema/channel
        # layout when recorded by a single ROS2 session.
        self.bag = self.make_reader(open(self.mcap_files[0], "rb"))
        self.summary = self.bag.get_summary()
        self.topic = self.check_topic(topic)
        self.n_scans = self._get_n_scans()
        self.msgs = self._iter_msgs()
        self.timestamps = []
        self.use_global_visualizer = True

    def __del__(self):
        if hasattr(self, "bag"):
            del self.bag

    def __getitem__(self, idx):
        msg = next(self.msgs).ros_msg
        self.timestamps.append(self.stamp_to_sec(msg.header.stamp))
        return self.read_point_cloud(msg)

    def __len__(self):
        return self.n_scans

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _collect_files(self, data_dir: str, topic: str) -> list:
        """Return .mcap files in *data_dir* sorted by first message timestamp.

        Falls back to natural filename order when the timestamp cannot be read
        from any file (e.g. the requested topic is not present in that file).
        Natural sort treats embedded numbers numerically so that run_10 comes
        after run_9 rather than after run_1.
        """

        def natural_sort_key(path: str):
            return [
                int(part) if part.isdigit() else part.lower()
                for part in re.split(r"(\d+)", path)
            ]

        candidates = sorted(
            glob.glob(os.path.join(str(data_dir), "*.mcap")),
            key=natural_sort_key,
        )
        assert len(candidates) > 0, f"No .mcap files found in directory: {data_dir}"

        def first_log_time(filepath: str):
            """Return the log_time (ns) of the first message on *topic*, or None."""
            try:
                with open(filepath, "rb") as f:
                    reader = self.make_reader(f)
                    for _schema, channel, message in reader.iter_messages(topics=[topic]):
                        if channel.topic == topic:
                            return message.log_time
                # Topic not present in this file
                return None
            except Exception:
                return None

        log_times = [first_log_time(f) for f in candidates]

        if all(t is not None for t in log_times):
            ordered = [f for _, f in sorted(zip(log_times, candidates))]
            sort_method = "timestamp"
        else:
            missing = [
                os.path.basename(f) for f, t in zip(candidates, log_times) if t is None
            ]
            print(
                f"[McapDataloader] WARNING: could not read timestamps from "
                f"{missing}. Falling back to natural filename order."
            )
            ordered = candidates
            sort_method = "filename (fallback)"

        print(
            f"[McapDataloader] Reading {len(ordered)} file(s) from '{data_dir}' "
            f"(sorted by {sort_method}):"
        )
        for f in ordered:
            print(f"  {os.path.basename(f)}")

        return ordered

    def _get_n_scans(self) -> int:
        """Count total PointCloud2 messages across all files for this topic."""
        total = 0
        for mcap_file in self.mcap_files:
            with open(mcap_file, "rb") as f:
                summary = self.make_reader(f).get_summary()
            total += sum(
                count
                for channel_id, count in summary.statistics.channel_message_counts.items()
                if summary.channels[channel_id].topic == self.topic
            )
        return total

    def _iter_msgs(self):
        """Yield messages from all files sequentially."""
        for mcap_file in self.mcap_files:
            yield from self.read_ros2_messages(mcap_file, topics=[self.topic])

    def reset(self):
        self.timestamps = []
        self.bag = self.make_reader(open(self.mcap_files[0], "rb"))
        self.msgs = self._iter_msgs()

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return stamp.sec + float(stamp.nanosec) / 1e9

    def get_frames_timestamps(self) -> list:
        return self.timestamps

    def check_topic(self, topic: str) -> str:
        # Extract schema id from the .mcap file that encodes the PointCloud2 msg
        schema_id = [
            schema.id
            for schema in self.summary.schemas.values()
            if schema.name == "sensor_msgs/msg/PointCloud2"
        ][0]

        point_cloud_topics = [
            channel.topic
            for channel in self.summary.channels.values()
            if channel.schema_id == schema_id
        ]

        def print_available_topics_and_exit():
            print(50 * "-")
            for t in point_cloud_topics:
                print(f"--topic {t}")
            print(50 * "-")
            sys.exit(1)

        if topic and topic in point_cloud_topics:
            return topic
        if topic and topic not in point_cloud_topics:
            print(
                f'[ERROR] Dataset does not contain any msg with the topic name "{topic}". '
                "Please select one of the following topics with the --topic flag"
            )
            print_available_topics_and_exit()
        if len(point_cloud_topics) > 1:
            print(
                "Multiple sensor_msgs/msg/PointCloud2 topics available. "
                "Please select one of the following topics with the --topic flag"
            )
            print_available_topics_and_exit()
        if len(point_cloud_topics) == 0:
            print("[ERROR] Your dataset does not contain any sensor_msgs/msg/PointCloud2 topic")
        if len(point_cloud_topics) == 1:
            return point_cloud_topics[0]