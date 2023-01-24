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
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "Utils.hpp"
#include "kiss_icp/core/Preprocessing.hpp"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/point_cloud2_iterator.h"

namespace {
auto GetVelodyneTimestamps(const std::vector<Eigen::Vector3d> &points) {
    std::vector<double> timestamps;
    timestamps.reserve(points.size());
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const double yaw = -std::atan2(point.y(), point.x());
        timestamps.emplace_back(0.5 * (yaw / M_PI + 1.0));
    });
    return timestamps;
}
}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "kitti_helper_node");
    ros::NodeHandle nh;

    using sensor_msgs::PointCloud2;
    auto frame_publisher = nh.advertise<PointCloud2>("pointcloud_topic", 1);
    auto sub = nh.subscribe<PointCloud2>("/velodyne_points", 1, [&](const auto &msg) {
        const auto points = kiss_icp_ros::utils::PointCloud2ToEigen(msg);
        const auto frame = kiss_icp::CorrectKITTIScan(points);
        const auto timestamps = GetVelodyneTimestamps(frame);
        auto msg_corr = kiss_icp_ros::utils::EigenToPointCloud2(frame, timestamps, msg->header);
        frame_publisher.publish(msg_corr);
    });

    // Notify people this node is a hack
    ROS_WARN("You should not be using kitti2rosbag for experimenting with KISS-ICP");
    ROS_WARN("kitti2rosbag is not the oringal format of the data.");
    ROS_WARN("If you want to only run experiments use the python API instead.");

    ros::spin();

    return 0;
}
