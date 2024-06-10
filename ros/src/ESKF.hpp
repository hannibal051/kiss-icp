#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "SavitzkyGolay.hpp"

using namespace Eigen;

namespace gps_imu_eskf {

struct State {
    Vector3d position;
    Vector3d velocity;
    Quaterniond orientation;
    Vector3d gyro_bias;
    Vector3d accel_bias;
};

struct ErrorState {
    Vector3d delta_position;
    Vector3d delta_velocity;
    Vector3d delta_orientation; // Small angle approximation
    Vector3d delta_gyro_bias;
    Vector3d delta_accel_bias;
};

class ESKFNode : public rclcpp::Node {
public:
    ESKFNode(const rclcpp::NodeOptions & options) : Node("eskf_node", options) {
        // These params are setup for re-localization
        gps_initialized_ = declare_parameter<bool>("origin_set", gps_initialized_);
        ORGIN_POINT_LAT = declare_parameter<double>("origin_lat", ORGIN_POINT_LAT);
        ORGIN_POINT_LON = declare_parameter<double>("origin_lon", ORGIN_POINT_LON);
        ORGIN_POINT_ALT = declare_parameter<double>("origin_alt", ORGIN_POINT_ALT);

        base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
        odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);

        std::string imu_topic_;
        imu_topic_ = declare_parameter<std::string>("imuTopic", imu_topic_);
        std::string gps_top_topic_;
        gps_top_topic_ = declare_parameter<std::string>("antennaTopTopic", gps_top_topic_);
        std::string gps_tbm_topic_;
        gps_tbm_topic_ = declare_parameter<std::string>("antennaBtmTopic", gps_tbm_topic_);
        std::string ins_topic_;
        ins_topic_ = declare_parameter<std::string>("insTopic", ins_topic_);
        std::string eskf_topic_;
        eskf_topic_ = declare_parameter<std::string>("eskfTopic", eskf_topic_);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 800, std::bind(&ESKFNode::imuCallback, this, std::placeholders::_1));
        gps_top_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_top_topic_, 10, std::bind(&ESKFNode::gpsTopCallback, this, std::placeholders::_1));
        gps_btm_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_tbm_topic_, 10, std::bind(&ESKFNode::gpsBtmCallback, this, std::placeholders::_1));
        ins_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            ins_topic_, 10, std::bind(&ESKFNode::insOdomCallback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(eskf_topic_, 1);

        // Initial nominal state
        nominal_state_.position = Vector3d(0, 0, 0);
        nominal_state_.velocity = Vector3d(0, 0, 0);
        nominal_state_.orientation = Quaterniond(1, 0, 0, 0);
        nominal_state_.gyro_bias = Vector3d(0, 0, 0);
        nominal_state_.accel_bias = Vector3d(0, 0, 0);
        // Initial SG smoother
        sg_smoother_ = SavitzkyGolay(5, 2);

        // Initial error-state covariance
        P_ = MatrixXd::Identity(15, 15); // 15x15 for position, velocity, orientation, gyro_bias, accel_bias
        // Initialize state covariance matrix P
        P_.block<3, 3>(0, 0) = Matrix3d::Identity() * 0.01;    // Position uncertainty (1 cm)
        P_.block<3, 3>(3, 3) = Matrix3d::Identity() * 0.1;     // Velocity uncertainty (initial guess)
        P_.block<3, 3>(6, 6) = Matrix3d::Identity() * 0.001;   // Orientation uncertainty (initial guess)
        P_.block<3, 3>(9, 9) = Matrix3d::Identity() * 1e-4;    // Gyro bias uncertainty (initial guess)
        P_.block<3, 3>(12, 12) = Matrix3d::Identity() * 1e-4;  // Accel bias uncertainty (initial guess)

        // Process noise covariance
        Q_ = MatrixXd::Identity(15, 15) * 0.01;
        Q_.block<3, 3>(0, 0) = Matrix3d::Identity() * pow(0.000392, 2) * 800;   // Position process noise
        Q_.block<3, 3>(3, 3) = Matrix3d::Identity() * pow(0.000392, 2) * 800;   // Velocity process noise
        Q_.block<3, 3>(6, 6) = Matrix3d::Identity() * pow(0.087266, 2) * 800;   // Orientation process noise
        Q_.block<3, 3>(9, 9) = Matrix3d::Identity() * pow(9.81e-5, 2);          // Gyro bias process noise
        Q_.block<3, 3>(12, 12) = Matrix3d::Identity() * pow(0.017453, 2);       // Accel bias process noise

        // IMU noise covariance
        Qi_ = MatrixXd::Zero(12, 12);
        Qi_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2;
        Qi_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-4;
        Qi_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-6;
        Qi_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-8;

        // Measurement noise covariance
        R_ = Matrix3d::Identity() * 0.01 * 0.01;
        R_yaw = MatrixXd::Identity(1, 1) * 0.01 * 0.01;

        // Initialize transformations from GPS antennas to base_link
        gps_front_to_base_link_ = Vector3d(+1.606, +0.05, -0.1428);
        gps_rear_to_base_link_ = Vector3d(+1.606, -0.05, -0.1428);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void insOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void gpsTopCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void gpsBtmCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    bool healthyGps(std::vector<Vector3d> & msg_buff, const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void publishOdometry();

    State applyErrorState(const State& nominal, const ErrorState& error);

    ErrorState integrateIMU(const State& nominal, const Vector3d& accel, const Vector3d& gyro, double dt);
    Matrix3d skewSymmetric(const Vector3d& v);

    Vector3d gpsToENU(double latitude, double longitude, double altitude, double lat_stdev = 5., double lon_stdev = 5., double alt_stdev = 5.);

    inline void transformGPSPositionToBaseLink(Vector3d& gps_position, const Vector3d& gps_to_base_link, const Quaterniond& orientation) {
        Vector3d transformed_vector = orientation * gps_to_base_link;
        gps_position -= transformed_vector;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_top_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_btm_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    State nominal_state_;
    MatrixXd P_, Q_, Qi_, R_yaw;
    Matrix3d R_;
    double last_imu_time_ = -1;
    rclcpp::Time msg_time_;

    SavitzkyGolay sg_smoother_;

    Vector3d gps_front_to_base_link_;
    Vector3d gps_rear_to_base_link_;

    std::vector<Vector3d> v_front_buff, v_rear_buff;

    bool gps_initialized_ = false;
    std::vector<double> vlat, vlon;
    double lon_sum, lat_sum, h_sum;

    double ORGIN_POINT_LAT;
    double ORGIN_POINT_LON;
    double ORGIN_POINT_ALT;
    double origin_yaw_;
    double ORGIN_POINT_LON_LENGTH;
    double ORGIN_POINT_LAT_LENGTH;

    /// Global/map coordinate frame.
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
};

} // namespace gps_imu_eskf