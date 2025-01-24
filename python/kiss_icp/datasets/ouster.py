# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
# Copyright (c) 2023 Pavlo Bashmakov
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
from typing import Optional

import numpy as np


class OusterDataloader:
    """Ouster pcap dataloader"""

    def __init__(
        self,
        data_dir: str,
        meta: Optional[str] = None,
        *_,
        **__,
    ):
        """Create Ouster pcap dataloader to read scans from a pcap file.

        Ouster pcap can be recorded with a `tcpdump` command or programmatically.
        Pcap file should contain raw lidar_packets and `meta` file (i.e. metadata.json)
        should be a corresponding sensor metadata stored at the time of pcap recording.


        NOTE: It's critical to have a metadata json stored in the same recording session
        as a pcap file, because pcap reader checks the `init_id` field in the UDP
        lidar_packets and expects it to match `initialization_id`
        in the metadata json, packets with different `init_id` just skipped.

        Metadata json can be obtainer with Ouster SDK:
        See examples here https://static.ouster.dev/sdk-docs/python/examples/basics-sensor.html#obtaining-sensor-metadata

        or with Sensor HTTP API endpoint GET /api/v1/sensor/metadata directly:
        See doc for details https://static.ouster.dev/sensor-docs/image_route1/image_route2/common_sections/API/http-api-v1.html#get-api-v1-sensor-metadata

        Args:
            data_dir: path to a pcap file (not a directory)
            meta: path to a metadata json file that should be recorded together with
            a pcap file. If `meta` is not provided attempts to find the best matching
            json file with the longest commong prefix of the pcap file (`data_dir`) in
            the same directory.
        """

        try:
            from ouster.sdk import client, open_source
        except ImportError:
            print(f'ouster-sdk is not installed on your system, run "pip install ouster-sdk"')
            exit(1)

        assert os.path.isfile(data_dir), "Ouster pcap dataloader expects an existing PCAP file"

        # we expect `data_dir` param to be a path to the .pcap file, so rename for clarity
        pcap_file = data_dir

        print("Indexing Ouster pcap to count the scans number ...")
        source = open_source(str(pcap_file), meta=[meta] if meta else [], index=True)

        # since we import ouster-sdk's client module locally, we keep reference
        # to it locally as well
        self._client = client

        self.data_dir = os.path.dirname(data_dir)

        # lookup table for 2D range image projection to a 3D point cloud
        self._xyz_lut = client.XYZLut(source.metadata)

        self._pcap_file = str(data_dir)

        self._scans_num = len(source)
        print(f"Ouster pcap total scans number:  {self._scans_num}")

        # frame timestamps array
        self._timestamps = np.linspace(0, self._scans_num, self._scans_num, endpoint=False)

        self._source = source

    def __getitem__(self, idx):
        scan = self._source[idx]

        self._timestamps[idx] = 1e-9 * scan.timestamp[0]

        timestamps = np.tile(np.linspace(0, 1.0, scan.w, endpoint=False), (scan.h, 1))

        # filtering our zero returns makes it substantially faster for kiss-icp
        sel_flag = scan.field(self._client.ChanField.RANGE) != 0
        xyz = self._xyz_lut(scan)[sel_flag]
        timestamps = timestamps[sel_flag]

        return xyz, timestamps

    def get_frames_timestamps(self):
        return self._timestamps

    def __len__(self):
        return self._scans_num
