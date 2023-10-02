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
    child_frame_ = declare_parameter<std::string>("child_frame", child_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_alias_tf_ = declare_parameter<bool>("publish_alias_tf", publish_alias_tf_);
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
    rclcpp::QoS qos(rclcpp::KeepLast{queue_size_});
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
    frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("frame", qos);
    kpoints_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("keypoints", qos);
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", qos);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize trajectory publisher
    path_msg_.header.frame_id = odom_frame_;
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", qos);

    // Broadcast a static transformation that links with identity the specified base link to the
    // pointcloud_frame, basically to always be able to visualize the frame in rviz
    if (publish_alias_tf_ && child_frame_ != "base_link") {
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions{
            rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History,
            rclcpp::QosPolicyKind::Reliability};
        options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

        static auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(
            *this, tf2_ros::StaticBroadcasterQoS(), options);

        geometry_msgs::msg::TransformStamped alias_transform_msg;
        alias_transform_msg.header.stamp = this->get_clock()->now();
        alias_transform_msg.transform.translation.x = 0.0;
        alias_transform_msg.transform.translation.y = 0.0;
        alias_transform_msg.transform.translation.z = 0.0;
        alias_transform_msg.transform.rotation.x = 0.0;
        alias_transform_msg.transform.rotation.y = 0.0;
        alias_transform_msg.transform.rotation.z = 0.0;
        alias_transform_msg.transform.rotation.w = 1.0;
        alias_transform_msg.header.frame_id = child_frame_;
        alias_transform_msg.child_frame_id = "base_link";
        br->sendTransform(alias_transform_msg);
    }

    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // PublishPose
    const auto pose = odometry_.poses().back();

    // Convert from Eigen to ROS types
    const Eigen::Vector3d t_current = pose.translation();
    const Eigen::Quaterniond q_current = pose.unit_quaternion();

    // Broadcast the tf
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = msg->header.stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = child_frame_;
        transform_msg.transform.rotation.x = q_current.x();
        transform_msg.transform.rotation.y = q_current.y();
        transform_msg.transform.rotation.z = q_current.z();
        transform_msg.transform.rotation.w = q_current.w();
        transform_msg.transform.translation.x = t_current.x();
        transform_msg.transform.translation.y = t_current.y();
        transform_msg.transform.translation.z = t_current.z();
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.orientation.x = q_current.x();
    pose_msg.pose.orientation.y = q_current.y();
    pose_msg.pose.orientation.z = q_current.z();
    pose_msg.pose.orientation.w = q_current.w();
    pose_msg.pose.position.x = t_current.x();
    pose_msg.pose.position.y = t_current.y();
    pose_msg.pose.position.z = t_current.z();
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = odom_frame_;
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // publish odometry msg
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header = pose_msg.header;
    odom_msg->child_frame_id = child_frame_;
    odom_msg->pose.pose = pose_msg.pose;
    odom_publisher_->publish(std::move(odom_msg));

    // Publish KISS-ICP internal data, just for debugging
    auto frame_header = msg->header;
    frame_header.frame_id = child_frame_;
    frame_publisher_->publish(std::move(EigenToPointCloud2(frame, frame_header)));
    kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, frame_header)));

    // Map is referenced to the odometry_frame
    auto local_map_header = msg->header;
    local_map_header.frame_id = odom_frame_;
    map_publisher_->publish(std::move(EigenToPointCloud2(odometry_.LocalMap(), local_map_header)));
}
}  // namespace kiss_icp_ros
