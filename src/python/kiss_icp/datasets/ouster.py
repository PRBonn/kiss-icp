import os
from typing import Optional

import numpy as np


class OusterDataloader:
    """Ouster pcap dataloader"""

    def __init__(
        self,
        data_dir: str,
        meta: str,
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
            a pcap file.
        """

        try:
            from ouster import client
            import ouster.pcap as pcap
        except ImportError:
            print(f'ouster-sdk is not installed on your system, run "pip install ouster-sdk"')
            exit(1)

        # we expect `data_dir` param to be a path to the .pcap file
        self.data_dir = os.path.dirname(data_dir)

        with open(meta) as json:
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
            "Ouster pcap Dataloader supports only sequential reads. "
            f"Expected idx: {self._next_idx}, but got {idx}"
        )
        scan = next(self._scans_iter)
        self._next_idx += 1

        if not self._timestamps_initialized:
            self._timestamps = np.tile(np.linspace(0, 1.0, scan.w, endpoint=False), scan.h)
            self._timestamps_initialized = True

        return self._xyz_lut(scan).reshape((-1, 3)), self._timestamps

    def __len__(self):
        return self._scans_num
