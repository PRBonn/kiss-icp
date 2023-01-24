// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <string>
#include <vector>

namespace kiss_icp_ros::utils {

// Some Ouster bagfiles have an annoying "/" at the begining of the frame_id which for whatever
// reason rviz does not like it. More info at https://github.com/ouster-lidar/ouster_example/pull/56
std::string FixFrameId(const std::string &frame_id);

/// Extract the timestamps values from the point cloud msg. The user can specify the timestamp
/// field, if non given, "t" and "timestamp" fields will be checked if present in the msg
std::vector<double> GetTimestamps(const sensor_msgs::PointCloud2ConstPtr &msg,
                                  const sensor_msgs::PointField &field = {});

/// Convert PointCloud2 msg to vector of Eigen
std::vector<Eigen::Vector3d> PointCloud2ToEigen(const sensor_msgs::PointCloud2ConstPtr &msg);

/// Convert vector of Eigen to PointCloud2 msg
sensor_msgs::PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                            const std_msgs::Header &header = {});

/// Convert vector of (Eigen, timestamps) to PointCloud2 msg. For the moment we only support
/// timestamps in the velodyne-like format, in the range [0.0, 1.0)
sensor_msgs::PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                            const std::vector<double> &timestamps,
                                            const std_msgs::Header &header = {});

}  // namespace kiss_icp_ros::utils
