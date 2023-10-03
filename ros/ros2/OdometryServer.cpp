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
#include <memory>
#include <sophus/se3.hpp>
#include <tuple>
#include <utility>
#include <vector>

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2 headers
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace kiss_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    // clang-format off
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    config_.max_range = declare_parameter<double>("max_range", config_.max_range);
    config_.min_range = declare_parameter<double>("min_range", config_.min_range);
    config_.deskew = declare_parameter<bool>("deskew", config_.deskew);
    config_.voxel_size = declare_parameter<double>("voxel_size", config_.max_range / 100.0);
    config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    config_.initial_threshold = declare_parameter<double>("initial_threshold", config_.initial_threshold);
    config_.min_motion_th = declare_parameter<double>("min_motion_th", config_.min_motion_th);
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }
    // clang-format on

    // Construct the main KISS-ICP odometry node
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/kiss/odometry", qos);
    frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/frame", qos);
    kpoints_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/keypoints", qos);
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/local_map", qos);
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("/kiss/trajectory", qos);
    path_msg_.header.frame_id = odom_frame_;

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
}

Sophus::SE3d OdometryServer::BaseLinkToCloudTf(const std::string &cloud_frame_id) {
    try {
        if (tf2_buffer_->_frameExists(base_frame_) && tf2_buffer_->_frameExists(cloud_frame_id) &&
            tf2_buffer_->canTransform(base_frame_, cloud_frame_id, tf2::TimePointZero)) {
            auto tf = tf2_buffer_->lookupTransform(base_frame_, cloud_frame_id, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return {};
    }
    // Should never reach this point, but if it does, spit the Identity
    RCLCPP_ERROR(this->get_logger(), "OdometryServer::BaseLinkToCloudTf failed to find tf");
    return {};
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    // Extract the points from the message in the desirded coordinate frame.
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    const auto [points, cloud_frame_id] = [&]() -> std::tuple<Vector3dVector, std::string> {
        // Extract point cloud and frame id from message
        auto points = PointCloud2ToEigen(msg);
        auto cloud_frame_id = msg->header.frame_id;

        if (base_frame_.empty() || base_frame_ == cloud_frame_id) {
            // Point cloud not modified, therefore it still expressed in its original frame_id
            return {points, cloud_frame_id};
        }

        // We need to express the input point cloud seen from the base coordinate frame
        const auto base2cloud = BaseLinkToCloudTf(cloud_frame_id);
        std::transform(points.cbegin(), points.cend(), points.begin(),
                       [&](const auto &point) { return base2cloud * point; });

        // Now the point cloud is in the base_frame_id, not the original one
        return {points, base_frame_};
    }();

    // Extract timestamps from the message for deskewing the cloud
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // PublishPose
    const Sophus::SE3d pose = odometry_.poses().back();

    // Header for point clouds and stuff seen from the cloud_frame_id (base_frame/original_frame)
    std_msgs::msg::Header cloud_header;
    cloud_header.stamp = msg->header.stamp;
    cloud_header.frame_id = cloud_frame_id;

    // Header for point clouds and stuff seen from desired odom_frame
    std_msgs::msg::Header odom_header;
    odom_header.stamp = msg->header.stamp;
    odom_header.frame_id = odom_frame_;

    // Broadcast the tf
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header = odom_header;
        transform_msg.child_frame_id = cloud_frame_id;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom_header;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // publish odometry msg
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header = odom_header;
    odom_msg->pose.pose = pose_msg.pose;
    odom_publisher_->publish(std::move(odom_msg));

    // Publish KISS-ICP internal data, just for debugging
    frame_publisher_->publish(std::move(EigenToPointCloud2(frame, cloud_header)));
    kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, cloud_header)));
    map_publisher_->publish(std::move(EigenToPointCloud2(odometry_.LocalMap(), odom_header)));
}
}  // namespace kiss_icp_ros
