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
#include <sophus/se2.hpp>
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

namespace kiss_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;
using utils::stamp2Sec;
using utils::quaternionToEulerAngles;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    bool use_lidar_;
    use_lidar_ = declare_parameter<bool>("useLidar", use_lidar_);
    if ( !use_lidar_ ) {
        return;
    }
    
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("visualize", publish_debug_clouds_);

    // These params are setup for re-localization
    echo_gps = declare_parameter<bool>("useImuHeadingInitialization", echo_gps);

    kiss_icp::pipeline::KISSConfig config;
    config.max_range = declare_parameter<double>("lidarMaxRange", config.max_range);
    config.min_range = declare_parameter<double>("lidarMinRange", config.min_range);
    config.deskew = declare_parameter<bool>("deskew", config.deskew);
    config.voxel_size = declare_parameter<double>("voxel_size", config.max_range / 100.0);
    config.max_points_per_voxel =
        declare_parameter<int>("max_points_per_voxel", config.max_points_per_voxel);
    config.initial_threshold =
        declare_parameter<double>("initial_threshold", config.initial_threshold);
    config.min_motion_th = declare_parameter<double>("min_motion_th", config.min_motion_th);
    if (config.max_range < config.min_range) {
        RCLCPP_WARN(get_logger(),
                    "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config.min_range = 0.0;
    }

    std::string pointcloud_topic;
    pointcloud_topic = declare_parameter<std::string>("pointCloudTopic", pointcloud_topic);
    std::string gps_filter_topic;
    gps_filter_topic = declare_parameter<std::string>("eskfTopic", gps_filter_topic);

    // Construct the main KISS-ICP odometry node
    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config);

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
    if ( echo_gps ) {
        gps_filter_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            gps_filter_topic, rclcpp::SensorDataQoS(),
            std::bind(&OdometryServer::GPSHandler, this, std::placeholders::_1));
    }

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/kiss/odometry", qos);

    if (publish_debug_clouds_) {
        frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/frame", qos);
        kpoints_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/keypoints", qos);
        local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/local_map", qos);
        global_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/global_map", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
}

Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame) const {
    std::string err_msg;
    if (tf2_buffer_->_frameExists(source_frame) &&  //
        tf2_buffer_->_frameExists(target_frame) &&  //
        tf2_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }
    RCLCPP_WARN(this->get_logger(), "Failed to find tf. Reason=%s", err_msg.c_str());
    return {};
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = GetTimestamps(msg);
    const auto egocentric_estimation = true;//(base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Find reference odometry from GPS data
    timeLaserInfoCur = stamp2Sec(msg->header.stamp);
    Sophus::SE3d test;  bool gpsRefered = ReferGPS(test);

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = kiss_icp_->RegisterFrame(points, timestamps, test, gpsRefered);

    // Compute the pose using KISS, ego-centric to the LiDAR
    const Sophus::SE3d kiss_pose = kiss_icp_->poses().back();

    // If necessary, transform the ego-centric pose to the specified base_link/base_footprint frame
    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return kiss_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * kiss_pose * cloud2base.inverse();
    }();

    // Spit the current estimated pose to ROS msgs
    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);
    // Publishing this clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(frame, keypoints, msg->header.stamp, cloud_frame_id);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // Broadcast the tf ---
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // publish odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::PublishClouds(const std::vector<Eigen::Vector3d> frame,
                                   const std::vector<Eigen::Vector3d> keypoints,
                                   const rclcpp::Time &stamp,
                                   const std::string &cloud_frame_id) {
    static int glob_cnt = -1;
    glob_cnt++;

    std_msgs::msg::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    // Publish map
    const auto kiss_map = kiss_icp_->LocalMap();

    if (!publish_odom_tf_) {
        // debugging happens in an egocentric world
        std_msgs::msg::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        frame_publisher_->publish(std::move(EigenToPointCloud2(frame, cloud_header)));
        kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, cloud_header)));
        local_map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, odom_header)));

        // if ( glob_cnt % 10 == 0 ) {
        //     const auto kiss_global_map = kiss_icp_->GlobalMap();
        //     global_map_publisher_->publish(std::move(EigenToPointCloud2(kiss_global_map, odom_header)));
        // }

        return;
    }

    // If transmitting to tf tree we know where the clouds are exactly
    const auto cloud2odom = LookupTransform(odom_frame_, cloud_frame_id);
    frame_publisher_->publish(std::move(EigenToPointCloud2(frame, cloud2odom, odom_header)));
    kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, cloud2odom, odom_header)));

    if (!base_frame_.empty()) {
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        local_map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, cloud2base, odom_header)));
    } else {
        local_map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, odom_header)));
    }

    // if ( glob_cnt % 10 == 0 ) {
    //     const auto kiss_global_map = kiss_icp_->GlobalMap();
    //     global_map_publisher_->publish(std::move(EigenToPointCloud2(kiss_global_map, odom_header)));
    // }
}

void OdometryServer::GPSHandler(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
    static bool init_rtk = false;
    static Sophus::SE2d origin;

    if (!init_rtk)
    {
        double cov = std::max(msg->pose.covariance[0], msg->pose.covariance[5]);

        double map_yaw = quaternionToEulerAngles(msg->pose.pose.orientation.w,
                                                 msg->pose.pose.orientation.x,
                                                 msg->pose.pose.orientation.y,
                                                 msg->pose.pose.orientation.z);
        double map_x = msg->pose.pose.position.x;
        double map_y = msg->pose.pose.position.y;
        
        // Create a rotation matrix from yaw
        // Sophus::SO2d R(Eigen::Rotation2Dd(map_yaw + 0.5 * M_PI).angle());
        Sophus::SO2d R(Eigen::Rotation2Dd(map_yaw).angle());
        // Create a translation vector
        Eigen::Vector2d t(map_x, map_y);

        // Create the Sophus SE2 group element
        Sophus::SE2d cur(R, t);
        origin = cur.inverse();
        
        // if ( cov <= 0.02 )
        {
            init_rtk = true;
        }
        
        return;
    }
    else
    {  
        double cov = std::max(msg->pose.covariance[0], msg->pose.covariance[5]);

        double yaw = quaternionToEulerAngles(msg->pose.pose.orientation.w,
                                             msg->pose.pose.orientation.x,
                                             msg->pose.pose.orientation.y,
                                             msg->pose.pose.orientation.z);

        // Create the recent SE2
        Sophus::SO2d R(Eigen::Rotation2Dd(yaw).angle());
        Eigen::Vector2d t(msg->pose.pose.position.x, msg->pose.pose.position.y);
        Sophus::SE2d curSE2(R, t);
        // get the final result
        Sophus::SE2d identity = origin * curSE2;
        // Extract the translation and rotation from the identity transformation
        Eigen::Vector2d trans = identity.translation();
        double final_yaw = identity.so2().log();
        
        if ( cov <= 0.5 )  // 0.02
        {
            nav_msgs::msg::Odometry pose_msg;
            pose_msg.pose.covariance[0] = cov;
            pose_msg.pose.covariance[7] = cov;
            pose_msg.pose.covariance[14] = cov;
            pose_msg.pose.pose.position.x = trans(0);
            pose_msg.pose.pose.position.y = trans(1);
            pose_msg.pose.pose.position.z = final_yaw;
            pose_msg.header.stamp = msg->header.stamp;

            gpsQueue.push_back(pose_msg);
        }
    }
}

bool OdometryServer::ReferGPS(Sophus::SE3d &test) {    
    while ( !gpsQueue.empty() ) {
        double curGPSt = stamp2Sec(gpsQueue.front().header.stamp);

        if (curGPSt < timeLaserInfoCur - 0.1) {     // 0.2
            gpsQueue.pop_front();
        }
        else if (curGPSt > timeLaserInfoCur + 0.1) {
            return false;
        }
        else if (curGPSt >= timeLaserInfoCur) {
            nav_msgs::msg::Odometry curGPS = gpsQueue.front();

            // Set the translation component of the SE3 object
            Eigen::Vector3d translation_vector(curGPS.pose.pose.position.x,
                                               curGPS.pose.pose.position.y,
                                               0.0);
            test.translation() = translation_vector;

            // Set the rotation component of the SE3 object
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = Eigen::AngleAxisd(curGPS.pose.pose.position.z, Eigen::Vector3d::UnitZ());
            test.setRotationMatrix(rotation_matrix);

            return true;
        }
        else {
            nav_msgs::msg::Odometry preGPS = gpsQueue.front();
            gpsQueue.pop_front();

            if ( !gpsQueue.empty() ) {
                nav_msgs::msg::Odometry curGPS = gpsQueue.front();
                double preGPSt = stamp2Sec(preGPS.header.stamp);
                double dt1 = timeLaserInfoCur - preGPSt;
                double curGPSt = stamp2Sec(curGPS.header.stamp);
                double dt2 = curGPSt - timeLaserInfoCur;

                if( dt1 > 0. && dt2 > 0. && dt2 < 0.1 ) {
                    // Set the translation component of the SE3 object
                    Eigen::Vector3d translation_vector((curGPS.pose.pose.position.x * dt1 + preGPS.pose.pose.position.x * dt2) / (dt1 + dt2),
                                                       (curGPS.pose.pose.position.y * dt1 + preGPS.pose.pose.position.y * dt2) / (dt1 + dt2),
                                                       0.0);
                    test.translation() = translation_vector;

                    // Set the rotation component of the SE3 object
                    Eigen::Matrix3d rotation_matrix;
                    rotation_matrix = Eigen::AngleAxisd((curGPS.pose.pose.position.z * dt1 + preGPS.pose.pose.position.z * dt2) / (dt1 + dt2), 
                                                        Eigen::Vector3d::UnitZ());
                    test.setRotationMatrix(rotation_matrix);
                }
                else {
                    // Set the translation component of the SE3 object
                    Eigen::Vector3d translation_vector(curGPS.pose.pose.position.x,
                                                       curGPS.pose.pose.position.y,
                                                       0.0);
                    test.translation() = translation_vector;

                    // Set the rotation component of the SE3 object
                    Eigen::Matrix3d rotation_matrix;
                    rotation_matrix = Eigen::AngleAxisd(curGPS.pose.pose.position.z, Eigen::Vector3d::UnitZ());
                    test.setRotationMatrix(rotation_matrix);
                }
            }
            else {
                // Set the translation component of the SE3 object
                Eigen::Vector3d translation_vector(preGPS.pose.pose.position.x,
                                                   preGPS.pose.pose.position.y,
                                                   0.0);
                test.translation() = translation_vector;

                // Set the rotation component of the SE3 object
                Eigen::Matrix3d rotation_matrix;
                rotation_matrix = Eigen::AngleAxisd(preGPS.pose.pose.position.z, Eigen::Vector3d::UnitZ());
                test.setRotationMatrix(rotation_matrix);
            }

            return true;
        }
    }

    return false;
}

}  // namespace kiss_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiss_icp_ros::OdometryServer)
