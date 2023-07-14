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
import glob
import os
from collections import namedtuple
from pathlib import Path

import numpy as np

__raw_to_odometry_mapping__ = {
    0: "2011_10_03/2011_10_03_drive_0027_sync/",
    1: "2011_10_03/2011_10_03_drive_0042_sync/",
    2: "2011_10_03/2011_10_03_drive_0034_sync/",
    4: "2011_09_30/2011_09_30_drive_0016_sync/",
    5: "2011_09_30/2011_09_30_drive_0018_sync/",
    6: "2011_09_30/2011_09_30_drive_0020_sync/",
    7: "2011_09_30/2011_09_30_drive_0027_sync/",
    8: "2011_09_30/2011_09_30_drive_0028_sync/",
    9: "2011_09_30/2011_09_30_drive_0033_sync/",
    10: "2011_09_30/2011_09_30_drive_0034_sync/",
}


class KITTIRawDataset:
    def __init__(self, data_dir: Path, sequence: int, *_, **__):
        self.sequence_id = str(int(sequence)).zfill(2)
        self.root_dir = os.path.realpath(data_dir / __raw_to_odometry_mapping__[sequence])
        self.date_id = self.root_dir.split("/")[-2]
        self.valid_idx = self.get_benchmark_indices(self.sequence_id)

        self.velodyne_dir = os.path.join(self.root_dir, "velodyne_points/data/")
        scan_files = sorted(glob.glob(self.velodyne_dir + "*.bin"))
        self.scan_files = scan_files[self.valid_idx[0] : self.valid_idx[1] + 1]
        self.calib_path = os.path.join(data_dir, f"{self.date_id}")
        self.calibration = self._load_calib()

        # IMU stuff
        self.oxts_dir = os.path.join(self.root_dir, "oxts/data/")
        oxts_files = sorted(glob.glob(self.oxts_dir + "*.txt"))
        self.oxts_files = oxts_files[self.valid_idx[0] : self.valid_idx[1] + 1]
        self.oxts, self.imu_poses = self.load_oxts_packets_and_poses(self.oxts_files)

        # GT poses in Velodyne frame
        self.gt_poses = self.imu_pose_to_lidar(self.imu_poses)

        # Add correction for KITTI datasets, can be easilty removed if unwanted
        from kiss_icp.pybind import kiss_icp_pybind

        self.correct_kitti_scan = lambda frame: np.asarray(
            kiss_icp_pybind._correct_kitti_scan(kiss_icp_pybind._Vector3dVector(frame))
        )

    def __len__(self):
        return len(self.scan_files)

    def __getitem__(self, idx):
        return self.read_point_cloud(self.scan_files[idx])

    def read_point_cloud(self, scan_file: str):
        points = np.fromfile(scan_file, dtype=np.float32).reshape((-1, 4))[:, :3].astype(np.float64)
        #  points = points[points[:, 2] > -2.9]  # Remove the annoying reflections
        points = self.correct_kitti_scan(points)
        return points, self.get_timestamps(points)

    def get_linear_velocity(self, idx):
        packet = self.oxts[idx].packet
        return np.array([packet.vf, packet.vl, packet.vu])

    def get_angular_velocity(self, idx):
        packet = self.oxts[idx].packet
        return np.array([packet.wf, packet.wl, packet.wu])

    def get_velocities(self, idx):
        return self.get_linear_velocity(idx), self.get_angular_velocity(idx)

    def imu_pose_to_lidar(self, poses):
        """Convert the imu poses to velodyne frame."""
        T_velo_imu = self.calibration["T_velo_imu"]
        T_imu_velo = np.linalg.inv(T_velo_imu)
        return T_velo_imu @ poses @ T_imu_velo

    @staticmethod
    def get_timestamps(points):
        x = points[:, 0]
        y = points[:, 1]
        yaw = -np.arctan2(y, x)
        timestamps = 0.5 * (yaw / np.pi + 1.0)
        return timestamps

    @staticmethod
    def get_benchmark_indices(sequence_id: str):
        _raw_to_benchmark_indices = {
            "00": [0, 4540],
            "01": [0, 1100],
            "02": [0, 4660],
            "04": [0, 270],
            "05": [0, 2760],
            "06": [0, 1100],
            "07": [0, 1100],
            "08": [1100, 5170],
            "09": [0, 1590],
            "10": [0, 1200],
        }
        return _raw_to_benchmark_indices[sequence_id]

    ### FROM THIS POINT EVERYTHING IS COPY PASTED FROM PYKITTI

    @staticmethod
    def read_calib_file(filepath):
        """Read in a calibration file and parse into a dictionary."""
        data = {}

        with open(filepath, "r") as f:
            for line in f.readlines():
                key, value = line.split(":", 1)
                # The only non-float values in these files are dates, which
                # we don't care about anyway
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass

        return data

    @staticmethod
    def transform_from_rot_trans(R, t):
        """Transforation matrix from rotation matrix and translation vector."""
        R = R.reshape(3, 3)
        t = t.reshape(3, 1)
        return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))

    def _load_calib_rigid(self, filename):
        """Read a rigid transform calibration file as a numpy.array."""
        filepath = os.path.join(self.calib_path, filename)
        data = self.read_calib_file(filepath)
        return self.transform_from_rot_trans(data["R"], data["T"])

    def _load_calib_cam_to_cam(self, velo_to_cam_file, cam_to_cam_file):
        # We'll return the camera calibration as a dictionary
        data = {}

        # Load the rigid transformation from velodyne coordinates
        # to unrectified cam0 coordinates
        T_cam0unrect_velo = self._load_calib_rigid(velo_to_cam_file)
        data["T_cam0_velo_unrect"] = T_cam0unrect_velo

        # Load and parse the cam-to-cam calibration data
        cam_to_cam_filepath = os.path.join(self.calib_path, cam_to_cam_file)
        filedata = self.read_calib_file(cam_to_cam_filepath)

        # Create 3x4 projection matrices
        P_rect_00 = np.reshape(filedata["P_rect_00"], (3, 4))
        P_rect_10 = np.reshape(filedata["P_rect_01"], (3, 4))
        P_rect_20 = np.reshape(filedata["P_rect_02"], (3, 4))
        P_rect_30 = np.reshape(filedata["P_rect_03"], (3, 4))

        data["P_rect_00"] = P_rect_00
        data["P_rect_10"] = P_rect_10
        data["P_rect_20"] = P_rect_20
        data["P_rect_30"] = P_rect_30

        # Create 4x4 matrices from the rectifying rotation matrices
        R_rect_00 = np.eye(4)
        R_rect_00[0:3, 0:3] = np.reshape(filedata["R_rect_00"], (3, 3))
        R_rect_10 = np.eye(4)
        R_rect_10[0:3, 0:3] = np.reshape(filedata["R_rect_01"], (3, 3))
        R_rect_20 = np.eye(4)
        R_rect_20[0:3, 0:3] = np.reshape(filedata["R_rect_02"], (3, 3))
        R_rect_30 = np.eye(4)
        R_rect_30[0:3, 0:3] = np.reshape(filedata["R_rect_03"], (3, 3))

        data["R_rect_00"] = R_rect_00
        data["R_rect_10"] = R_rect_10
        data["R_rect_20"] = R_rect_20
        data["R_rect_30"] = R_rect_30

        # Compute the rectified extrinsics from cam0 to camN
        T0 = np.eye(4)
        T0[0, 3] = P_rect_00[0, 3] / P_rect_00[0, 0]
        T1 = np.eye(4)
        T1[0, 3] = P_rect_10[0, 3] / P_rect_10[0, 0]
        T2 = np.eye(4)
        T2[0, 3] = P_rect_20[0, 3] / P_rect_20[0, 0]
        T3 = np.eye(4)
        T3[0, 3] = P_rect_30[0, 3] / P_rect_30[0, 0]

        # Compute the velodyne to rectified camera coordinate transforms
        data["T_cam0_velo"] = T0.dot(R_rect_00.dot(T_cam0unrect_velo))
        data["T_cam1_velo"] = T1.dot(R_rect_00.dot(T_cam0unrect_velo))
        data["T_cam2_velo"] = T2.dot(R_rect_00.dot(T_cam0unrect_velo))
        data["T_cam3_velo"] = T3.dot(R_rect_00.dot(T_cam0unrect_velo))

        # Compute the camera intrinsics
        data["K_cam0"] = P_rect_00[0:3, 0:3]
        data["K_cam1"] = P_rect_10[0:3, 0:3]
        data["K_cam2"] = P_rect_20[0:3, 0:3]
        data["K_cam3"] = P_rect_30[0:3, 0:3]

        # Compute the stereo baselines in meters by projecting the origin of
        # each camera frame into the velodyne frame and computing the distances
        # between them
        p_cam = np.array([0, 0, 0, 1])
        p_velo0 = np.linalg.inv(data["T_cam0_velo"]).dot(p_cam)
        p_velo1 = np.linalg.inv(data["T_cam1_velo"]).dot(p_cam)
        p_velo2 = np.linalg.inv(data["T_cam2_velo"]).dot(p_cam)
        p_velo3 = np.linalg.inv(data["T_cam3_velo"]).dot(p_cam)

        data["b_gray"] = np.linalg.norm(p_velo1 - p_velo0)  # gray baseline
        data["b_rgb"] = np.linalg.norm(p_velo3 - p_velo2)  # rgb baseline

        return data

    def _load_calib(self):
        """Load and compute intrinsic and extrinsic calibration parameters."""
        # We'll build the calibration parameters as a dictionary, then
        # convert it to a namedtuple to prevent it from being modified later
        data = {}

        # Load the rigid transformation from IMU to velodyne
        data["T_velo_imu"] = self._load_calib_rigid("calib_imu_to_velo.txt")

        # Load the camera intrinsics and extrinsics
        data.update(self._load_calib_cam_to_cam("calib_velo_to_cam.txt", "calib_cam_to_cam.txt"))

        # Pre-compute the IMU to rectified camera coordinate transforms
        data["T_cam0_imu"] = data["T_cam0_velo"].dot(data["T_velo_imu"])
        data["T_cam1_imu"] = data["T_cam1_velo"].dot(data["T_velo_imu"])
        data["T_cam2_imu"] = data["T_cam2_velo"].dot(data["T_velo_imu"])
        data["T_cam3_imu"] = data["T_cam3_velo"].dot(data["T_velo_imu"])

        return data

    @staticmethod
    def pose_from_oxts_packet(packet, scale):
        """Helper method to compute a SE(3) pose matrix from an OXTS packet."""

        def rotx(t):
            """Rotation about the x-axis."""
            c = np.cos(t)
            s = np.sin(t)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

        def roty(t):
            """Rotation about the y-axis."""
            c = np.cos(t)
            s = np.sin(t)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

        def rotz(t):
            """Rotation about the z-axis."""
            c = np.cos(t)
            s = np.sin(t)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        er = 6378137.0  # earth radius (approx.) in meters

        # Use a Mercator projection to get the translation vector
        tx = scale * packet.lon * np.pi * er / 180.0
        ty = scale * er * np.log(np.tan((90.0 + packet.lat) * np.pi / 360.0))
        tz = packet.alt
        t = np.array([tx, ty, tz])

        # Use the Euler angles to get the rotation matrix
        Rx = rotx(packet.roll)
        Ry = roty(packet.pitch)
        Rz = rotz(packet.yaw)
        R = Rz.dot(Ry.dot(Rx))

        # Combine the translation and rotation into a homogeneous transform
        return R, t

    def load_oxts_packets_and_poses(self, oxts_files):
        """Generator to read OXTS ground truth data.

        Poses are given in an East-North-Up coordinate system
        whose origin is the first GPS position.

        GPS/IMU 3D localization unit
        ============================

        The GPS/IMU information is given in a single small text file which is
        written for each synchronized frame. Each text file contains 30 values
        which are:

          - lat:     latitude of the oxts-unit (deg)
          - lon:     longitude of the oxts-unit (deg)
          - alt:     altitude of the oxts-unit (m)
          - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
          - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
          - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
          - vn:      velocity towards north (m/s)
          - ve:      velocity towards east (m/s)
          - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
          - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
          - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
          - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
          - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
          - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
          - af:      forward acceleration (m/s^2)
          - al:      leftward acceleration (m/s^2)
          - au:      upward acceleration (m/s^2)
          - wx:      angular rate around x (rad/s)
          - wy:      angular rate around y (rad/s)
          - wz:      angular rate around z (rad/s)
          - wf:      angular rate around forward axis (rad/s)
          - wl:      angular rate around leftward axis (rad/s)
          - wu:      angular rate around upward axis (rad/s)
          - posacc:  velocity accuracy (north/east in m)
          - velacc:  velocity accuracy (north/east in m/s)
          - navstat: navigation status
          - numsats: number of satellites tracked by primary GPS receiver
          - posmode: position mode of primary GPS receiver
          - velmode: velocity mode of primary GPS receiver
          - orimode: orientation mode of primary GPS receiver

        To read the text file and interpret them properly an example is given in
        the matlab folder: First, use oxts = loadOxtsliteData('2011_xx_xx_drive_xxxx')
        to read in the GPS/IMU data. Next, use pose = convertOxtsToPose(oxts) to
        transform the oxts data into local euclidean poses, specified by 4x4 rigid
        transformation matrices. For more details see the comments in those files.

        """
        # Per dataformat.txt
        OxtsPacket = namedtuple(
            "OxtsPacket",
            "lat, lon, alt, "
            + "roll, pitch, yaw, "
            + "vn, ve, vf, vl, vu, "
            + "ax, ay, az, af, al, au, "
            + "wx, wy, wz, wf, wl, wu, "
            + "pos_accuracy, vel_accuracy, "
            + "navstat, numsats, "
            + "posmode, velmode, orimode",
        )

        # Bundle into an easy-to-access structure
        OxtsData = namedtuple("OxtsData", "packet, T_w_imu")
        # Scale for Mercator projection (from first lat value)
        scale = None
        # Origin of the global coordinate system (first GPS position)
        origin = None

        oxts = []
        T_w_imu_poses = []

        for filename in oxts_files:
            with open(filename, "r") as f:
                for line in f.readlines():
                    line = line.split()
                    # Last five entries are flags and counts
                    line[:-5] = [float(x) for x in line[:-5]]
                    line[-5:] = [int(float(x)) for x in line[-5:]]

                    packet = OxtsPacket(*line)

                    if scale is None:
                        scale = np.cos(packet.lat * np.pi / 180.0)

                    R, t = self.pose_from_oxts_packet(packet, scale)

                    if origin is None:
                        origin = t

                    T_w_imu = self.transform_from_rot_trans(R, t)
                    T_w_imu_poses.append(T_w_imu)

                    oxts.append(OxtsData(packet, T_w_imu))

        # Start from identity
        first_pose = T_w_imu_poses[0]
        T_w_imu_poses = np.linalg.inv(first_pose) @ T_w_imu_poses
        return oxts, T_w_imu_poses
