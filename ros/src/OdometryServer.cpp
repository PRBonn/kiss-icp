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
#include <memory>
#include <sophus/se3.hpp>
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
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
namespace {
Sophus::SE3d LookupTransform(const std::string &target_frame,
                             const std::string &source_frame,
                             const std::unique_ptr<tf2_ros::Buffer> &tf2_buffer) {
    std::string err_msg;
    if (tf2_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "%s", ex.what());
        }
    }
    RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "Failed to find tf. Reason=%s",
                err_msg.c_str());
    // default construction is the identity
    return Sophus::SE3d();
}
}  // namespace

namespace kiss_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("kiss_icp_node", options) {
    kiss_icp::pipeline::KISSConfig config;
    initializeParameters(config);

    // Construct the main KISS-ICP odometry node
    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config);
    
    // Enable metrics collection if adaptive covariance is enabled OR metrics_only_mode is enabled
    // (use_adaptive_covariance_ and metrics_only_mode_ were set in initializeParameters)
    kiss_icp_->setUseRegistrationMetrics(use_adaptive_covariance_ || metrics_only_mode_);
    
    if (metrics_only_mode_) {
        RCLCPP_INFO(get_logger(), "Registration metrics collection ENABLED (metrics-only mode - fixed covariance)");
    } else if (use_adaptive_covariance_) {
        RCLCPP_INFO(get_logger(), "Registration metrics collection ENABLED (adaptive covariance)");
    } else {
        RCLCPP_INFO(get_logger(), "Registration metrics collection DISABLED (using original method)");
    }
    
    // Initialize metrics publisher if metrics collection is enabled
    if (use_adaptive_covariance_ || metrics_only_mode_) {
        rclcpp::QoS metrics_qos((rclcpp::SystemDefaultsQoS().keep_last(10).durability_volatile()));
        metrics_pub_ = create_publisher<kiss_icp::msg::RegistrationMetrics>("kiss/metrics", metrics_qos);
        RCLCPP_INFO(get_logger(), "Metrics publisher initialized: /kiss/metrics");
    }

    process_each_x_frame_ = declare_parameter<int>("process_each_x_frame", 1);
    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("kiss/odometry", qos);
    if (publish_debug_clouds_) {
        frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("kiss/frame", qos);
        kpoints_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("kiss/keypoints", qos);
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("kiss/local_map", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
    // Initialize service servers
    reset_service_ = create_service<std_srvs::srv::Empty>(
        "kiss/reset", std::bind(&OdometryServer::ResetService, this, std::placeholders::_1,
                                std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
}

void OdometryServer::initializeParameters(kiss_icp::pipeline::KISSConfig &config) {
    RCLCPP_INFO(this->get_logger(), "Initializing parameters");

    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    RCLCPP_INFO(this->get_logger(), "\tBase frame: %s", base_frame_.c_str());
    lidar_odom_frame_ = declare_parameter<std::string>("lidar_odom_frame", lidar_odom_frame_);
    RCLCPP_INFO(this->get_logger(), "\tLiDAR odometry frame: %s", lidar_odom_frame_.c_str());
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    RCLCPP_INFO(this->get_logger(), "\tPublish odometry transform: %d", publish_odom_tf_);
    invert_odom_tf_ = declare_parameter<bool>("invert_odom_tf", invert_odom_tf_);
    RCLCPP_INFO(this->get_logger(), "\tInvert odometry transform: %d", invert_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", publish_debug_clouds_);
    RCLCPP_INFO(this->get_logger(), "\tPublish debug clouds: %d", publish_debug_clouds_);
    position_covariance_ = declare_parameter<double>("position_covariance", 0.1);
    RCLCPP_INFO(this->get_logger(), "\tPosition covariance: %.2f", position_covariance_);
    orientation_covariance_ = declare_parameter<double>("orientation_covariance", 0.1);
    RCLCPP_INFO(this->get_logger(), "\tOrientation covariance: %.2f", orientation_covariance_);
    
    // Adaptive covariance parameters
    use_adaptive_covariance_ = declare_parameter<bool>("adaptive_covariance.enable", false);
    RCLCPP_INFO(this->get_logger(), "\tUse adaptive covariance: %d", use_adaptive_covariance_);
    
    metrics_only_mode_ = declare_parameter<bool>("adaptive_covariance.metrics_only_mode", false);
    RCLCPP_INFO(this->get_logger(), "\tMetrics only mode: %d", metrics_only_mode_);
    
    // Only read other adaptive params if enabled or metrics_only_mode
    if (use_adaptive_covariance_ || metrics_only_mode_) {
        nominal_keypoint_count_ = declare_parameter<double>("adaptive_covariance.nominal_keypoints", 1000.0);
        RCLCPP_INFO(this->get_logger(), "\tNominal keypoint count: %.0f", nominal_keypoint_count_);
        
        min_keypoint_ratio_ = declare_parameter<double>("adaptive_covariance.min_ratio", 0.5);
        RCLCPP_INFO(this->get_logger(), "\tMin keypoint ratio: %.2f", min_keypoint_ratio_);
        
        max_covariance_multiplier_ = declare_parameter<double>("adaptive_covariance.max_multiplier", 10.0);
        RCLCPP_INFO(this->get_logger(), "\tMax covariance multiplier: %.1fx", max_covariance_multiplier_);
        
        enable_covariance_smoothing_ = declare_parameter<bool>("adaptive_covariance.enable_smoothing", true);
        RCLCPP_INFO(this->get_logger(), "\tEnable covariance smoothing: %d", enable_covariance_smoothing_);
        
        covariance_smoothing_alpha_ = declare_parameter<double>("adaptive_covariance.smoothing_alpha", 0.3);
        RCLCPP_INFO(this->get_logger(), "\tCovariance smoothing alpha: %.2f", covariance_smoothing_alpha_);
    }

    config.max_range = declare_parameter<double>("data.max_range", config.max_range);
    RCLCPP_INFO(this->get_logger(), "\tMax range: %.2f", config.max_range);
    config.min_range = declare_parameter<double>("data.min_range", config.min_range);
    RCLCPP_INFO(this->get_logger(), "\tMin range: %.2f", config.min_range);
    config.deskew = declare_parameter<bool>("data.deskew", config.deskew);
    RCLCPP_INFO(this->get_logger(), "\tDeskew: %d", config.deskew);
    config.voxel_size = declare_parameter<double>("mapping.voxel_size", config.max_range / 100.0);
    RCLCPP_INFO(this->get_logger(), "\tVoxel size: %.2f", config.voxel_size);
    config.max_points_per_voxel =
        declare_parameter<int>("mapping.max_points_per_voxel", config.max_points_per_voxel);
    RCLCPP_INFO(this->get_logger(), "\tMax points per voxel: %d", config.max_points_per_voxel);
    config.initial_threshold =
        declare_parameter<double>("adaptive_threshold.initial_threshold", config.initial_threshold);
    RCLCPP_INFO(this->get_logger(), "\tInitial threshold: %.2f", config.initial_threshold);
    config.min_motion_th =
        declare_parameter<double>("adaptive_threshold.min_motion_th", config.min_motion_th);
    RCLCPP_INFO(this->get_logger(), "\tMin motion threshold: %.2f", config.min_motion_th);
    config.max_num_iterations =
        declare_parameter<int>("registration.max_num_iterations", config.max_num_iterations);
    RCLCPP_INFO(this->get_logger(), "\tMax number of iterations: %d", config.max_num_iterations);
    config.convergence_criterion = declare_parameter<double>("registration.convergence_criterion",
                                                             config.convergence_criterion);
    RCLCPP_INFO(this->get_logger(), "\tConvergence criterion: %.2f", config.convergence_criterion);
    config.max_num_threads =
        declare_parameter<int>("registration.max_num_threads", config.max_num_threads);
    RCLCPP_INFO(this->get_logger(), "\tMax number of threads: %d", config.max_num_threads);
    if (config.max_range < config.min_range) {
        RCLCPP_WARN(get_logger(),
                    "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config.min_range = 0.0;
    }
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (frames_count_++ % process_each_x_frame_ != 0) {
        return;
    }
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = GetTimestamps(msg);

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = kiss_icp_->RegisterFrame(points, timestamps);

    // Extract the last KISS-ICP pose, ego-centric to the LiDAR
    const Sophus::SE3d kiss_pose = kiss_icp_->pose();

    // Spit the current estimated pose to ROS msgs handling the desired target frame
    PublishOdometry(kiss_pose, msg->header);
    // Publishing these clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(frame, keypoints, msg->header);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &kiss_pose,
                                     const std_msgs::msg::Header &header) {
    // If necessary, transform the ego-centric pose to the specified base_link/base_footprint frame
    const auto cloud_frame_id = header.frame_id;
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);
    const auto moving_frame = egocentric_estimation ? cloud_frame_id : base_frame_;
    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return kiss_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id, tf2_buffer_);
        return cloud2base * kiss_pose * cloud2base.inverse();
    }();

    // Broadcast the tf ---
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = header.stamp;
        if (invert_odom_tf_) {
            transform_msg.header.frame_id = moving_frame;
            transform_msg.child_frame_id = lidar_odom_frame_;
            transform_msg.transform = tf2::sophusToTransform(pose.inverse());
        } else {
            transform_msg.header.frame_id = lidar_odom_frame_;
            transform_msg.child_frame_id = moving_frame;
            transform_msg.transform = tf2::sophusToTransform(pose);
        }
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // Get registration quality metrics (only if enabled)
    size_t num_correspondences = 0;
    size_t num_source_points = 0;
    double cov_multiplier = 1.0;
    
    if (use_adaptive_covariance_ || metrics_only_mode_) {
        num_correspondences = kiss_icp_->num_correspondences();
        num_source_points = kiss_icp_->num_source_points();
        
        // Publish metrics for statistical analysis
        if (metrics_pub_) {
            kiss_icp::msg::RegistrationMetrics metrics_msg;
            metrics_msg.num_correspondences = num_correspondences;
            metrics_msg.num_source_points = num_source_points;
            metrics_pub_->publish(metrics_msg);
        }
        
        // Compute multiplier (will be 1.0 if metrics_only_mode is true)
        cov_multiplier = computeCovarianceMultiplier(num_correspondences, num_source_points);
    }
    
    // Publish odometry msg with adaptive covariance
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = lidar_odom_frame_;
    odom_msg.child_frame_id = moving_frame;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_msg.pose.covariance.fill(0.0);
    
    // Apply multiplier (will be 1.0 if disabled)
    double adaptive_pos_cov = position_covariance_ * cov_multiplier;
    double adaptive_orient_cov = orientation_covariance_ * cov_multiplier;
    
    odom_msg.pose.covariance[0] = adaptive_pos_cov;    // x
    odom_msg.pose.covariance[7] = adaptive_pos_cov;     // y
    odom_msg.pose.covariance[14] = adaptive_pos_cov;    // z
    odom_msg.pose.covariance[21] = adaptive_orient_cov; // roll
    odom_msg.pose.covariance[28] = adaptive_orient_cov; // pitch
    odom_msg.pose.covariance[35] = adaptive_orient_cov; // yaw
    
    // Optional logging for debugging (only when enabled)
    if (use_adaptive_covariance_ || metrics_only_mode_) {
        RCLCPP_DEBUG(get_logger(), 
                     "Metrics: corr=%zu/%zu (%.1f%%), mult=%.2fx, pos_cov=%.3f, orient_cov=%.3f%s",
                     num_correspondences, num_source_points,
                     100.0 * num_correspondences / std::max(size_t(1), num_source_points),
                     cov_multiplier, adaptive_pos_cov, adaptive_orient_cov,
                     metrics_only_mode_ ? " [METRICS-ONLY]" : "");
    }
    
    odom_publisher_->publish(std::move(odom_msg));
}

double OdometryServer::computeCovarianceMultiplier(size_t num_correspondences, 
                                                    size_t num_source_points) {
    // If metrics_only_mode is enabled, always return 1.0 (fixed covariance)
    if (metrics_only_mode_) {
        return 1.0;
    }
    
    // If adaptive covariance is disabled, return 1.0 (no change)
    // Note: use_adaptive_covariance_ controls both the ROS parameter and the internal metrics collection
    if (!use_adaptive_covariance_) {
        return 1.0;
    }
    
    if (num_source_points == 0) {
        return max_covariance_multiplier_;  // Worst case
    }
    
    // Inlier ratio: how many source points actually matched, in [0, 1]
    const double inlier_ratio = static_cast<double>(num_correspondences) /
                                static_cast<double>(num_source_points);

    // Absolute correspondences quality: how many inliers vs expected minimum
    double correspondence_quality = static_cast<double>(num_correspondences) / nominal_keypoint_count_ * min_keypoint_ratio_;

    // Clamp to [0, 1]
    correspondence_quality = std::clamp(correspondence_quality, 0.0, 1.0);

    // 4. Combine inlier ratio + absolute quality into a single quality factor
    //    Simple product: both have to be good to get a high score.
    double quality_factor = correspondence_quality * inlier_ratio;

    // Safety clamp
    quality_factor = std::clamp(quality_factor, 0.0, 1.0);

    // 5. Map quality [0,1] -> multiplier [1, max_covariance_multiplier_]
    //    Qubic mapping.
    const double error = 1.0 - quality_factor;      // in [0, 1]
    double multiplier = 1.0 + (max_covariance_multiplier_ - 1.0) * std::pow(error, 3);

    // 6. Optional exponential smoothing to reduce jitter frame-to-frame
    if (enable_covariance_smoothing_) {
        smoothed_covariance_multiplier_ = covariance_smoothing_alpha_ * multiplier +
            (1.0 - covariance_smoothing_alpha_) * smoothed_covariance_multiplier_;
        multiplier = smoothed_covariance_multiplier_;
    }

    return multiplier;
}

void OdometryServer::PublishClouds(const std::vector<Eigen::Vector3d> &frame,
                                   const std::vector<Eigen::Vector3d> &keypoints,
                                   const std_msgs::msg::Header &header) {
    const auto kiss_map = kiss_icp_->LocalMap();

    frame_publisher_->publish(std::move(EigenToPointCloud2(frame, header)));
    kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, header)));
    auto local_map_header = header;
    local_map_header.frame_id = lidar_odom_frame_;
    map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, local_map_header)));
}
void OdometryServer::ResetService(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Resetting KISS-ICP map and odometry");

    // Reset the KISS-ICP pipeline
    kiss_icp_->Reset();

    RCLCPP_INFO(this->get_logger(), "KISS-ICP reset completed successfully");
}
}  // namespace kiss_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiss_icp_ros::OdometryServer)
