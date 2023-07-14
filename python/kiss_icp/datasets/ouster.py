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

import glob
import os
from typing import Optional

import numpy as np


def find_metadata_json(pcap_file: str) -> str:
    """Attempts to resolve the metadata json file for a provided pcap file."""
    dir_path, filename = os.path.split(pcap_file)
    if not filename:
        return ""
    if not dir_path:
        dir_path = os.getcwd()
    json_candidates = sorted(glob.glob(f"{dir_path}/*.json"))
    if not json_candidates:
        return ""
    prefix_sizes = list(
        map(lambda p: len(os.path.commonprefix((filename, os.path.basename(p)))), json_candidates)
    )
    max_elem = max(range(len(prefix_sizes)), key=lambda i: prefix_sizes[i])
    return json_candidates[max_elem]


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
            import ouster.pcap as pcap
            from ouster import client
        except ImportError:
            print(f'ouster-sdk is not installed on your system, run "pip install ouster-sdk"')
            exit(1)

        # since we import ouster-sdk's client module locally, we keep it locally as well
        self._client = client

        assert os.path.isfile(data_dir), "Ouster pcap dataloader expects an existing PCAP file"

        # we expect `data_dir` param to be a path to the .pcap file, so rename for clarity
        pcap_file = data_dir

        metadata_json = meta or find_metadata_json(pcap_file)
        if not metadata_json:
            print("Ouster pcap dataloader can't find metadata json file.")
            exit(1)
        print("Ouster pcap dataloader: using metadata json: ", metadata_json)

        self.data_dir = os.path.dirname(data_dir)

        with open(metadata_json) as json:
            self._info_json = json.read()
            self._info = client.SensorInfo(self._info_json)

        # lookup table for 2D range image projection to a 3D point cloud
        self._xyz_lut = client.XYZLut(self._info)

        self._pcap_file = str(data_dir)

        # read pcap file for the first pass to count scans
        print("Pre-reading Ouster pcap to count the scans number ...")
        self._source = pcap.Pcap(self._pcap_file, self._info)
        self._scans_num = sum((1 for _ in client.Scans(self._source)))
        print(f"Ouster pcap total scans number:  {self._scans_num}")

        # cached timestamps array
        self._timestamps = np.empty(0)
        self._timestamps_initialized = False

        # start Scans iterator for consumption in __getitem__
        self._source = pcap.Pcap(self._pcap_file, self._info)
        self._scans_iter = iter(client.Scans(self._source))
        self._next_idx = 0

    def __getitem__(self, idx):
        # we assume that users always reads sequentially and do not
        # pass idx as for a random access collection
        assert self._next_idx == idx, (
            "Ouster pcap dataloader supports only sequential reads. "
            f"Expected idx: {self._next_idx}, but got {idx}"
        )
        scan = next(self._scans_iter)
        self._next_idx += 1

        if not self._timestamps_initialized:
            self._timestamps = np.tile(np.linspace(0, 1.0, scan.w, endpoint=False), (scan.h, 1))
            self._timestamps_initialized = True

        # filtering our zero returns makes it substantially faster for kiss-icp
        sel_flag = scan.field(self._client.ChanField.RANGE) != 0
        xyz = self._xyz_lut(scan)[sel_flag]
        timestamps = self._timestamps[sel_flag]

        return xyz, timestamps

    def __len__(self):
        return self._scans_num
