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

// ROS 1 headers
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace kiss_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
    pnh_.param("base_frame", base_frame_, base_frame_);
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    pnh_.param("publish_odom_tf", publish_odom_tf_, false);
    pnh_.param("max_range", config_.max_range, config_.max_range);
    pnh_.param("min_range", config_.min_range, config_.min_range);
    pnh_.param("deskew", config_.deskew, config_.deskew);
    pnh_.param("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    pnh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    pnh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
    pnh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    if (config_.max_range < config_.min_range) {
        ROS_WARN("[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    // Construct the main KISS-ICP odometry node
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud_topic", queue_size_,
                                                              &OdometryServer::RegisterFrame, this);

    // Initialize publishers
    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/kiss/odometry", queue_size_);
    frame_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/kiss/frame", queue_size_);
    kpoints_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/kiss/keypoints", queue_size_);
    map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/kiss/local_map", queue_size_);
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("/kiss/trajectory", queue_size_);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
    path_msg_.header.frame_id = odom_frame_;

    // publish odometry msg
    ROS_INFO("KISS-ICP ROS 1 Odometry Node Initialized");
}

Sophus::SE3d OdometryServer::CloudToBaseTf(const std::string &cloud_frame_id) const {
    std::string err_msg;
    if (tf2_buffer_->_frameExists(base_frame_) &&     //
        tf2_buffer_->_frameExists(cloud_frame_id) &&  //
        tf2_buffer_->canTransform(cloud_frame_id, base_frame_, ros::Time(0), &err_msg)) {
        try {
            auto tf = tf2_buffer_->lookupTransform(cloud_frame_id, base_frame_, ros::Time(0));
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    ROS_ERROR("Failed to find tf between %s and %s. Reason=%s", 
        base_frame_.c_str(), cloud_frame_id.c_str(), err_msg.c_str());
    return {};
}

void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // Compute the pose using KISS, ego-centric to the LiDAR
    const Sophus::SE3d kiss_pose = odometry_.poses().back();

    // If necessary, transform the ego-centric pose to the specified base_link/base_footprint frame
    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return kiss_pose;
        const Sophus::SE3d cloud2base = CloudToBaseTf(cloud_frame_id);
        return cloud2base.inverse() * kiss_pose * cloud2base;
    }();

    // Header for point clouds and stuff seen from desired odom_frame
    std_msgs::Header odom_header = msg->header;
    odom_header.frame_id = odom_frame_;

    // Broadcast the tf
    if (publish_odom_tf_) {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header = odom_header;
        transform_msg.child_frame_id = egocentric_estimation ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = odom_header;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_.publish(path_msg_);

    // publish odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header = odom_header;
    odom_msg.pose.pose = pose_msg.pose;
    odom_publisher_.publish(odom_msg);


    // Publish KISS-ICP internal map, just for debugging
    const auto kiss_map = odometry_.LocalMap();
    if (egocentric_estimation) {
        map_publisher_.publish(*EigenToPointCloud2(kiss_map, odom_header));
    } else {
        const auto T = CloudToBaseTf(cloud_frame_id).inverse();
        map_publisher_.publish(*EigenToPointCloud2(kiss_map, T, odom_header));
    }
}
}  // namespace kiss_icp_ros

int main(int argc, char **argv) {
    ros::init(argc, argv, "kiss_icp");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    kiss_icp_ros::OdometryServer node(nh, nh_private);

    ros::spin();

    return 0;
}
