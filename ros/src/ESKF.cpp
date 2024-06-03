// KISS-ICP-ROS
#include "ESKF.hpp"
#include "Utils.hpp"

#define TWO_D

using namespace Eigen;

namespace gps_imu_eskf {

void ESKFNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if ( !gps_initialized_ ) {
        return;
    }

    msg_time_ = msg->header.stamp;

    // Convert IMU data to Eigen format
    Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // Perform IMU preintegration and update error state
    double imuTime = kiss_icp_ros::utils::stamp2Sec(msg_time_);
    double dt = (last_imu_time_ < 0) ? (1.0 / 500.0) : (imuTime - last_imu_time_);
    if ( dt <= 0 ) {
        return;
    }
    last_imu_time_ = imuTime;
    ErrorState error_state = integrateIMU(nominal_state_, accel, gyro, dt);

    // Update nominal state with the error state
    nominal_state_ = applyErrorState(nominal_state_, error_state);

    // Propagate the state covariance matrix P
    MatrixXd F = computeStateTransitionMatrix(nominal_state_, accel, gyro, dt);  // State transition matrix
    MatrixXd G = computeProcessNoiseJacobian(nominal_state_, accel, gyro, dt);  // Process noise Jacobian
    P_ = F * P_ * F.transpose() + G * Q_ * G.transpose();

    publishOdometry();
}

void ESKFNode::insOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract yaw from the INS odometry message
    Quaterniond ins_orientation(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    msg_time_ = msg->header.stamp;

    last_imu_time_ = kiss_icp_ros::utils::stamp2Sec(msg_time_);

    // Convert quaternion to Euler angles to get the yaw
    Vector3d euler = ins_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    double ins_yaw = euler.z();

    // Extract the yaw from the current orientation
    Vector3d current_euler = nominal_state_.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    double current_yaw = current_euler.z();

    // Compute the yaw innovation
    double yaw_innovation = ins_yaw - current_yaw;

    // Measurement matrix for yaw
    MatrixXd H = MatrixXd::Zero(1, 15);
    H(0, 8) = 1.0;

    // Measurement noise covariance for yaw (as a 1x1 matrix)
    MatrixXd R_yaw = MatrixXd::Identity(1, 1) * 0.1; // Adjust as necessary

    // Kalman gain for yaw
    MatrixXd S = H * P_ * H.transpose() + R_yaw;
    MatrixXd K = P_ * H.transpose() * S.inverse();

    // Update error-state covariance
    P_ = (MatrixXd::Identity(15, 15) - K * H) * P_;

    // Update error state with yaw innovation
    VectorXd delta_x = K * yaw_innovation;
    ErrorState correction;
    correction.delta_position = Vector3d::Zero();
    correction.delta_velocity = Vector3d::Zero();
    correction.delta_orientation = Vector3d(0, 0, delta_x(0)); // Only update yaw
    correction.delta_gyro_bias = Vector3d::Zero();
    correction.delta_accel_bias = Vector3d::Zero();

    // Apply correction to the nominal state (only update yaw orientation)
    nominal_state_ = applyErrorState(nominal_state_, correction);

    // Reset the error state to zero after correction
    P_ = (MatrixXd::Identity(15, 15) - K * H) * P_;

    // Publish the updated pose
    publishOdometry();
}

void ESKFNode::gpsFrontCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // Convert GPS data to ENU coordinates (this assumes you have a function for this conversion)
    Vector3d gps_position = gpsToENU(msg->latitude, msg->longitude, msg->altitude);
    if ( !gps_initialized_ ) {
        return;
    }

    v_front_buff.emplace_back(gps_position);
    if ( !healthyGps(v_front_buff, msg) ) {
        std::cout << "\033[38;5;196m !! Unhealthy Front GPS !! \033[0m" << std::endl;
        return;
    }

    transformGPSPositionToBaseLink(gps_position, gps_rear_to_base_link_, nominal_state_.orientation);

    msg_time_ = msg->header.stamp;

    last_imu_time_ = kiss_icp_ros::utils::stamp2Sec(msg_time_);

    // GPS measurement update
    Vector3d innovation = gps_position - nominal_state_.position;
#ifdef TWO_D
    innovation(2) = 0.;
#endif

    // Kalman gain
    MatrixXd H = MatrixXd::Zero(3, 15); // Measurement matrix
    H.block<3, 3>(0, 0) = Matrix3d::Identity();
    MatrixXd S = H * P_ * H.transpose() + R_;
    MatrixXd K = P_ * H.transpose() * S.inverse();

    K(0, 0) = 1.0;
    K(1, 1) = 1.0;
    K(2, 2) = 1.0;

    // Update error-state covariance
    P_ = (MatrixXd::Identity(15, 15) - K * H) * P_;

    // Update error state with innovation
    VectorXd delta_x = K * innovation;
    ErrorState correction;
    correction.delta_position = delta_x.segment<3>(0);
    correction.delta_velocity = Vector3d::Zero();
    correction.delta_orientation = Vector3d::Zero();
    correction.delta_gyro_bias = Vector3d::Zero();
    correction.delta_accel_bias = Vector3d::Zero();

    // Apply correction to the nominal state
    nominal_state_ = applyErrorState(nominal_state_, correction);

    // Reset the error state to zero after correction
    P_ = (MatrixXd::Identity(15, 15) - K * H) * P_;

    // Publish the updated pose
    publishOdometry();
}


void ESKFNode::gpsRearCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // Convert GPS data to ENU coordinates (this assumes you have a function for this conversion)
    Vector3d gps_position = gpsToENU(msg->latitude, msg->longitude, msg->altitude);
    if ( !gps_initialized_ ) {
        return;
    }

    v_rear_buff.emplace_back(gps_position);
    if ( !healthyGps(v_rear_buff, msg) ) {
        std::cout << "\033[38;5;196m !! Unhealthy Rear GPS !! \033[0m" << std::endl;
        return;
    }

    transformGPSPositionToBaseLink(gps_position, gps_rear_to_base_link_, nominal_state_.orientation);

    msg_time_ = msg->header.stamp;

    last_imu_time_ = kiss_icp_ros::utils::stamp2Sec(msg_time_);

    // GPS measurement update
    Vector3d innovation = gps_position - nominal_state_.position;
#ifdef TWO_D
    innovation(2) = 0.;
#endif

    // Kalman gain
    MatrixXd H = MatrixXd::Zero(3, 15); // Measurement matrix
    H.block<3, 3>(0, 0) = Matrix3d::Identity();
    MatrixXd S = H * P_ * H.transpose() + R_;
    MatrixXd K = P_ * H.transpose() * S.inverse();

    // Update error-state covariance
    P_ = (MatrixXd::Identity(15, 15) - K * H) * P_;

    K(0, 0) = 1.0;
    K(1, 1) = 1.0;
    K(2, 2) = 1.0;

    // Update error state with innovation
    VectorXd delta_x = K * innovation;
    ErrorState correction;
    correction.delta_position = delta_x.segment<3>(0);
    correction.delta_velocity = Vector3d::Zero();
    correction.delta_orientation = Vector3d::Zero();
    correction.delta_gyro_bias = Vector3d::Zero();
    correction.delta_accel_bias = Vector3d::Zero();

    // Apply correction to the nominal state
    nominal_state_ = applyErrorState(nominal_state_, correction);

    // Reset the error state to zero after correction
    P_ = (MatrixXd::Identity(15, 15) - K * H) * P_;

    // Publish the updated pose
    publishOdometry();
}

void ESKFNode::publishOdometry() {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = msg_time_;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = nominal_state_.position.x();
    odom_msg.pose.pose.position.y = nominal_state_.position.y();
    odom_msg.pose.pose.position.z = nominal_state_.position.z();
    odom_msg.pose.pose.orientation.x = nominal_state_.orientation.x();
    odom_msg.pose.pose.orientation.y = nominal_state_.orientation.y();
    odom_msg.pose.pose.orientation.z = nominal_state_.orientation.z();
    odom_msg.pose.pose.orientation.w = nominal_state_.orientation.w();
    odom_msg.twist.twist.linear.x = nominal_state_.velocity.x();
    odom_msg.twist.twist.linear.y = nominal_state_.velocity.y();
    odom_msg.twist.twist.linear.z = nominal_state_.velocity.z();

    // Fill covariance matrices (placeholders, replace with actual covariance if available)
    for (int i = 0; i < 6; ++i) {
        odom_msg.pose.covariance[i * 6 + i] = 0.1;  // Diagonal elements
        odom_msg.twist.covariance[i * 6 + i] = 0.1; // Diagonal elements
    }

    odom_pub_->publish(odom_msg);
}

State ESKFNode::applyErrorState(const State& nominal, const ErrorState& error) {
    State updated = nominal;
    updated.position += error.delta_position;
    updated.velocity += error.delta_velocity;
    updated.orientation = nominal.orientation * Quaterniond(AngleAxisd(error.delta_orientation.norm(), error.delta_orientation.normalized()));
    updated.gyro_bias += error.delta_gyro_bias;
    updated.accel_bias += error.delta_accel_bias;
    return updated;
}

ErrorState ESKFNode::integrateIMU(const State& nominal, const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Remove biases from IMU measurements
    Vector3d corrected_accel = accel - nominal.accel_bias;
    Vector3d corrected_gyro = gyro - nominal.gyro_bias;
    // Transform to ENU frame
    corrected_accel = Vector3d(corrected_accel(1), -corrected_accel(0), corrected_accel(2));
    corrected_gyro = Vector3d(corrected_gyro(1), -corrected_gyro(0), corrected_gyro(2));
    // Update velocity using the corrected acceleration measurements
#ifdef TWO_D
    corrected_accel(2) = 0.0;
    corrected_gyro(0) = 0.0;
    corrected_gyro(1) = 0.0;
#else
    corrected_accel(2) += 9.0005;
#endif

    // Update delta rotation
    Vector3d wdt = corrected_gyro * dt;
    double d = wdt.norm();
    Matrix3d deltaR;
    Matrix3d W = skewSymmetric(wdt);
    if(d < 1e-4)
    {
        deltaR = Matrix3d::Identity() + W;
    }
    else
    {
        deltaR = Matrix3d::Identity() + W*sin(d)/d + W*W*(1.0f-cos(d))/(d*d);
    }
    Vector3d euler = deltaR.eulerAngles(0, 1, 2);
    for (int i = 0; i < 3; ++i) {
        if (euler[i] > M_PI / 60.0) euler[i] -= M_PI;
        if (euler[i] < -M_PI / 60.0) euler[i] += M_PI;
        if (euler[i] > M_PI / 60.0 || euler[i] < -M_PI / 60.0) euler[i] = 0.0;
    }

    // Create the error state
    ErrorState error_state;
    error_state.delta_position = nominal.velocity * dt + 0.5f * corrected_accel * dt * dt;
    error_state.delta_velocity = corrected_accel * dt;
    error_state.delta_orientation = euler;
    error_state.delta_gyro_bias = Vector3d::Zero(); // Biases are constant over a short time step
    error_state.delta_accel_bias = Vector3d::Zero(); // Biases are constant over a short time step

    return error_state;
}

MatrixXd ESKFNode::computeStateTransitionMatrix(const State& nominal, const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Compute the state transition matrix F based on the current nominal state, accelerometer, and gyroscope measurements
    MatrixXd F = MatrixXd::Identity(15, 15);
    
    // Fill in the details for F based on the system dynamics
    // Example for position and velocity update parts
    F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;  // Position-velocity relationship
    F.block<3, 3>(3, 6) = -skewSymmetric(nominal.orientation.toRotationMatrix() * (accel - nominal.accel_bias)) * dt;  // Velocity-orientation relationship

    // Additional terms can be filled in similarly
    return F;
}

MatrixXd ESKFNode::computeProcessNoiseJacobian(const State& nominal, const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Compute the process noise Jacobian matrix G based on the current nominal state, accelerometer, and gyroscope measurements
    MatrixXd G = MatrixXd::Zero(15, 15);

    // Fill in the details for G based on the system dynamics and noise characteristics
    // Example for accelerometer and gyroscope noise effects on position and velocity
    G.block<3, 3>(0, 0) = Matrix3d::Identity() * dt;  // Accelerometer noise effect on position
    G.block<3, 3>(3, 3) = Matrix3d::Identity() * dt;  // Accelerometer noise effect on velocity
    G.block<3, 3>(6, 6) = Matrix3d::Identity() * dt;  // Gyroscope noise effect on orientation

    // Additional terms can be filled in similarly
    return G;
}

Matrix3d ESKFNode::skewSymmetric(const Vector3d& v) {
    Matrix3d S;
    S <<    0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),    0;
    return S;
}

Vector3d ESKFNode::gpsToENU(double latitude, double longitude, double altitude, double lat_stdev, double lon_stdev, double alt_stdev) {
    double north, east;
    std::string zone;
    kiss_icp_ros::utils::LLtoUTM(latitude, longitude, north, east, zone);

    if ( !gps_initialized_ ) {
        /*  Step 1
            If we can directly decide from raw messages
        */
        if ( lat_stdev < 0.05 && lon_stdev < 0.05 ) {
            gps_initialized_ = true;
            ORGIN_POINT_LON = longitude;
            ORGIN_POINT_LAT = latitude;
            ORGIN_POINT_ALT = altitude;

            double n1, e1, n2, e2;
            std::string zone1, zone2;
            kiss_icp_ros::utils::LLtoUTM(latitude, longitude, n1, e1, zone1);
            kiss_icp_ros::utils::LLtoUTM(latitude + 1.0, longitude, n2, e2, zone2);
            ORGIN_POINT_LAT_LENGTH = abs(n1 - n2);
            kiss_icp_ros::utils::LLtoUTM(latitude, longitude + 1.0, n2, e2, zone2);
            ORGIN_POINT_LON_LENGTH = abs(e1 - e2);
            return Vector3d(0.0, 0.0, 0.0);
        }

        /*  Step 2
            If we cannot trust raw messages fully
        */
        vlat.push_back(latitude);
        vlon.push_back(longitude);
        lon_sum += longitude;
        lat_sum += latitude;
        h_sum += altitude;

        if(vlon.size() > 3 && vlat.size() > 3) {
            double lat_mean = lat_sum / vlat.size();
            double lat_accum = 0.0;

            for(int i = 0; i < vlat.size(); i++) {
                lat_accum  += (vlat[i]-lat_mean)*(vlat[i]-lat_mean);
            }

            double lat_stdev = sqrt(lat_accum/(vlat.size()-1));

            double lon_mean =  lon_sum / vlon.size(); 
            double lon_accum  = 0.0;
            for(int i=0;i<vlon.size();i++){
                lon_accum  += (vlon[i]-lon_mean)*(vlon[i]-lon_mean);
            }
            double lon_stdev = sqrt(lon_accum/(vlon.size()-1));
            double h_mean = h_sum / vlon.size();
            if(lon_stdev < 0.00001 && lat_stdev < 0.00001) {
                gps_initialized_ = true;
                ORGIN_POINT_LON = lon_mean;
                ORGIN_POINT_LAT = lat_mean;
                ORGIN_POINT_ALT = h_mean;

                double n1, e1, n2, e2;
                std::string zone1, zone2;
                kiss_icp_ros::utils::LLtoUTM(latitude, longitude, n1, e1, zone1);
                kiss_icp_ros::utils::LLtoUTM(latitude + 1.0, longitude, n2, e2, zone2);
                ORGIN_POINT_LAT_LENGTH = abs(n1 - n2);
                kiss_icp_ros::utils::LLtoUTM(latitude, longitude + 1.0, n2, e2, zone2);
                ORGIN_POINT_LON_LENGTH = abs(e1 - e2);
                return Vector3d(0.0, 0.0, 0.0);
            }
            else {
                vlon.clear();
                vlat.clear();
                vlon.resize(0);
                vlat.resize(0);

                lon_sum = 0.0;
                lat_sum = 0.0;
                h_sum = 0.0;
                return Vector3d(0.0, 0.0, 0.0);;
            }
        }
    }

    double m_x = static_cast<double>((longitude - ORGIN_POINT_LON) * ORGIN_POINT_LON_LENGTH);
    double m_y = static_cast<double>((latitude - ORGIN_POINT_LAT) * ORGIN_POINT_LAT_LENGTH);
    double m_z = static_cast<double>(altitude - ORGIN_POINT_ALT);
    return Vector3d(m_x, m_y, m_z);
}

bool ESKFNode::healthyGps(std::vector<Vector3d> & msg_buff, const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if ( msg->position_covariance[0] > 0.2 
        || msg->position_covariance[4] > 0.2 
        || msg->position_covariance[8] > 0.2 
        || msg->status.status == -1 ) {
            return false;
    }

    int sizeOf = msg_buff.size();
    if ( sizeOf <= 1 ) {
        return true;
    }
    // TODO: consider delta-timestamps
    else if ( sizeOf == 2 ) {
        double dis = (msg_buff[1] - msg_buff[0]).norm();
        if ( dis > 8.0 ) {
            return false;
        }
    }
    else {
        double dis = (msg_buff[2] - msg_buff[1]).norm();
        double dis_mid = (msg_buff[2] + msg_buff[0] - msg_buff[1] * 2).norm();

        msg_buff.erase(msg_buff.begin());

        if ( dis > 8.0 || dis_mid > 8.0 ) {
            return false;
        }
    }
    return true;
}

} // namespace gps_imu_eskf

RCLCPP_COMPONENTS_REGISTER_NODE(gps_imu_eskf::ESKFNode)