import os
from typing import Optional

import numpy as np

from ouster import client
import ouster.pcap as pcap

class OusterDataloader:
    def __init__(
        self,
        data_dir,
        meta: Optional[str] = None,
        *_,
        **__,
    ):
        self.data_dir = os.path.dirname(data_dir)

        with open(meta) as json:
            self._info_json = json.read()
            self._info = client.SensorInfo(self._info_json)

        self._xyz_lut = client.XYZLut(self._info)

        self._pcap_file = str(data_dir)

        print("info = ", self._info)
        print("pcap_file = ", self._pcap_file)

        # read full Pcap beforehand to count scans count
        self._source = pcap.Pcap(self._pcap_file, self._info)
        self._scans_num = sum([1 for _ in client.Scans(self._source)])

        self._timestamps = np.empty(0)
        self._timestamps_initialized = False

        print("scans_num = ", self._scans_num)

        # start Scans iterator for consumption in __getitem__
        self._source = pcap.Pcap(self._pcap_file, self._info)
        self._scans_iter = iter(client.Scans(self._source))

    def __getitem__(self, idx):
        # TODO: handle only forward read by idx??? and raise error on something else ... 
        scan = next(self._scans_iter)
        if not self._timestamps_initialized:
            self._timestamps = np.tile(np.linspace(0, 1.0, scan.w, endpoint=False), scan.h)
            self._timestamps_initialized = True
        return self._xyz_lut(scan).reshape((-1, 3)), self._timestamps

    def __len__(self):
        # print(f"ouster:getitem, len = {len(self.scans)}")
        return self._scans_num