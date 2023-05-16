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

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include <fstream>

#include "kiss_icp/pipeline/KissICP.hpp"

// ROS
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace kiss_icp_ros {

OdometryServer::OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
    pnh_.param("child_frame", child_frame_, child_frame_);
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
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
    pointcloud_sub_ = nh_.subscribe<const sensor_msgs::PointCloud2 &>(
        "pointcloud_topic", queue_size_, &OdometryServer::RegisterFrame, this);

    // Initialize publishers
    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("odometry", queue_size_);
    frame_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("frame", queue_size_);
    kpoints_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("keypoints", queue_size_);
    local_map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("local_map", queue_size_);

    // Initialize trajectory publisher
    path_msg_.header.frame_id = odom_frame_;
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("trajectory", queue_size_);

    // Advertise save service
    save_traj_srv_ = pnh_.advertiseService("SaveTrajectory", &OdometryServer::SaveTrajectory, this);

    // Broadcast a static transformation that links with identity the specified base link to the
    // pointcloud_frame, basically to always be able to visualize the frame in rviz
    if (child_frame_ != "base_link") {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped alias_transform_msg;
        alias_transform_msg.header.stamp = ros::Time::now();
        alias_transform_msg.transform.translation.x = 0.0;
        alias_transform_msg.transform.translation.y = 0.0;
        alias_transform_msg.transform.translation.z = 0.0;
        alias_transform_msg.transform.rotation.x = 0.0;
        alias_transform_msg.transform.rotation.y = 0.0;
        alias_transform_msg.transform.rotation.z = 0.0;
        alias_transform_msg.transform.rotation.w = 1.0;
        alias_transform_msg.header.frame_id = child_frame_;
        alias_transform_msg.child_frame_id = "base_link";
        br.sendTransform(alias_transform_msg);
    }

    // publish odometry msg
    ROS_INFO("KISS-ICP ROS 1 Odometry Node Initialized");
}

void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2 &msg) {
    const auto points = utils::PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return utils::GetTimestamps(msg);
    }();

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // PublishPose
    const auto pose = odometry_.poses().back();

    // Convert from Eigen to ROS types
    const Eigen::Vector3d t_current = pose.translation();
    const Eigen::Quaterniond q_current = pose.unit_quaternion();

    // Broadcast the tf
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = msg.header.stamp;
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = child_frame_;
    transform_msg.transform.rotation.x = q_current.x();
    transform_msg.transform.rotation.y = q_current.y();
    transform_msg.transform.rotation.z = q_current.z();
    transform_msg.transform.rotation.w = q_current.w();
    transform_msg.transform.translation.x = t_current.x();
    transform_msg.transform.translation.y = t_current.y();
    transform_msg.transform.translation.z = t_current.z();
    tf_broadcaster_.sendTransform(transform_msg);

    // publish odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg.header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose.orientation.x = q_current.x();
    odom_msg.pose.pose.orientation.y = q_current.y();
    odom_msg.pose.pose.orientation.z = q_current.z();
    odom_msg.pose.pose.orientation.w = q_current.w();
    odom_msg.pose.pose.position.x = t_current.x();
    odom_msg.pose.pose.position.y = t_current.y();
    odom_msg.pose.pose.position.z = t_current.z();
    odom_publisher_.publish(odom_msg);

    // publish trajectory msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose = odom_msg.pose.pose;
    pose_msg.header = odom_msg.header;
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_.publish(path_msg_);

    // Publish KISS-ICP internal data, just for debugging
    std_msgs::Header frame_header = msg.header;
    frame_header.frame_id = child_frame_;
    frame_publisher_.publish(utils::EigenToPointCloud2(frame, frame_header));
    kpoints_publisher_.publish(utils::EigenToPointCloud2(keypoints, frame_header));

    // Map is referenced to the odometry_frame
    std_msgs::Header local_map_header = msg.header;
    local_map_header.frame_id = odom_frame_;
    local_map_publisher_.publish(utils::EigenToPointCloud2(odometry_.LocalMap(), local_map_header));
}

bool OdometryServer::SaveTrajectory(kiss_icp::SaveTrajectory::Request &path,
                                    kiss_icp::SaveTrajectory::Response &response) {
    std::string kittipath = path.path + "_kitti.txt";
    std::string tumpath = path.path + "_tum.txt";
    std::ofstream out_tum(tumpath);
    std::ofstream out_kitti(kittipath);
    for (const auto &pose : path_msg_.poses) {
        const auto &t = pose.pose.position;
        const auto &q = pose.pose.orientation;
        // Write to tum format, ts,x,y,z,qx,qy,qz,qw
        out_tum << std::fixed << std::setprecision(9) << pose.header.stamp << " " << t.x << " "
                << t.y << " " << t.z << " " << q.x << " " << q.y << " " << q.z << " " << q.w
                << "\n";
        // Write to Kitti Format
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        auto R = quat.normalized().toRotationMatrix();

        out_kitti << std::fixed << std::setprecision(9) << R(0, 0) << " " << R(0, 1) << " "
                  << R(0, 2) << " " << t.x << " " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2)
                  << " " << t.y << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t.z
                  << "\n";
    }
    ROS_INFO("Saved Trajectory");
    return true;
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
