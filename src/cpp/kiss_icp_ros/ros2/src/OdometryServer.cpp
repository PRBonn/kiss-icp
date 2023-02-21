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
#include <vector>

// KISS-ICP
#include "OdometryServer.hpp"
#include "Utils.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace kiss_icp_ros {

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    child_frame_ = get_parameter_or<std::string>("child_frame", child_frame_, child_frame_);
    odom_frame_ = get_parameter_or<std::string>("odom_frame", odom_frame_, odom_frame_);
    config_.max_range = get_parameter_or<double>("max_range", config_.max_range, config_.max_range);
    config_.min_range = get_parameter_or<double>("min_range", config_.min_range, config_.min_range);
    config_.deskew = get_parameter_or<bool>("deskew", config_.deskew, config_.deskew);
    config_.frame_rate =
        get_parameter_or<double>("frame_rate", config_.frame_rate, config_.frame_rate);
    config_.voxel_size =
        get_parameter_or<double>("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    config_.max_points_per_voxel = get_parameter_or<int>(
        "max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    config_.initial_threshold = get_parameter_or<double>(
        "initial_threshold", config_.initial_threshold, config_.initial_threshold);
    config_.min_motion_th =
        get_parameter_or<double>("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(),
                    "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }

    // Construct the main KISS-ICP odometry node
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Intialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));

    // Intialize publishers
    rclcpp::QoS qos(rclcpp::KeepLast{queue_size_});
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
    frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("frame", qos);
    kpoints_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("keypoints", qos);
    local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", qos);

    // Intialize trajectory publisher
    path_msg_.header.frame_id = odom_frame_;
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", qos);

    // Broadcast a static transformation that links with identity the specified base link to the
    // pointcloud_frame, basically to always be able to visualize the frame in rviz
    if (child_frame_ != "base_link") {
        auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        ;
        geometry_msgs::msg::TransformStamped alias_transform_msg;
        alias_transform_msg.header.stamp = now();
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

    // publish odometry msg
    RCLCPP_INFO(get_logger(), "KISS-ICP ROS 2 Odometry Node Unitialized");
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const auto &points = utils::PointCloud2ToEigen(msg);
    const auto &timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return utils::GetTimestamps(msg);
    }();

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // PublishPose
    const Eigen::Isometry3d pose = odometry_.poses().back();

    // Convert from Eigen to ROS types
    const Eigen::Vector3d t_current = pose.translation();
    const Eigen::Quaterniond q_current(pose.rotation());

    // Broadcast the tf
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

    // publish odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose.orientation.x = q_current.x();
    odom_msg.pose.pose.orientation.y = q_current.y();
    odom_msg.pose.pose.orientation.z = q_current.z();
    odom_msg.pose.pose.orientation.w = q_current.w();
    odom_msg.pose.pose.position.x = t_current.x();
    odom_msg.pose.pose.position.y = t_current.y();
    odom_msg.pose.pose.position.z = t_current.z();
    odom_publisher_->publish(odom_msg);

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose = odom_msg.pose.pose;
    pose_msg.header = odom_msg.header;
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // Publish KISS-ICP internal data, just for debugging
    std_msgs::msg::Header frame_header = msg->header;
    frame_header.frame_id = child_frame_;
    frame_publisher_->publish(utils::EigenToPointCloud2(frame, frame_header));
    kpoints_publisher_->publish(utils::EigenToPointCloud2(keypoints, frame_header));

    // Map is referenced to the odometry_frame
    std_msgs::msg::Header local_map_header = msg->header;
    local_map_header.frame_id = odom_frame_;
    local_map_publisher_->publish(
        utils::EigenToPointCloud2(odometry_.LocalMap(), local_map_header));
}

}  // namespace kiss_icp_ros
