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
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <regex>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

// ROS 2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace tf2 {

inline geometry_msgs::msg::Transform sophusToTransform(const Sophus::SE3d &T) {
    geometry_msgs::msg::Transform t;
    t.translation.x = T.translation().x();
    t.translation.y = T.translation().y();
    t.translation.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();

    return t;
}

inline geometry_msgs::msg::Pose sophusToPose(const Sophus::SE3d &T) {
    geometry_msgs::msg::Pose t;
    t.position.x = T.translation().x();
    t.position.y = T.translation().y();
    t.position.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.orientation.x = q.x();
    t.orientation.y = q.y();
    t.orientation.z = q.z();
    t.orientation.w = q.w();

    return t;
}

inline Sophus::SE3d transformToSophus(const geometry_msgs::msg::TransformStamped &transform) {
    const auto &t = transform.transform;
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
        Sophus::SE3d::Point(t.translation.x, t.translation.y, t.translation.z));
}
}  // namespace tf2

namespace kiss_icp_ros::utils {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

inline std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline std::optional<PointField> GetTimestampField(const PointCloud2::ConstSharedPtr msg) {
    PointField timestamp_field;
    for (const auto &field : msg->fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time")) {
            timestamp_field = field;
        }
    }
    if (timestamp_field.count) return timestamp_field;
    RCLCPP_WARN_ONCE(rclcpp::get_logger("odometry_node"),
                     "Field 't', 'timestamp', or 'time'  does not exist. "
                     "Disabling scan deskewing");
    return {};
}

// Normalize timestamps from 0.0 to 1.0
inline auto NormalizeTimestamps(const std::vector<double> &timestamps) {
    const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const double min_timestamp = *min_it;
    const double max_timestamp = *max_it;

    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const auto &timestamp) {
                       return (timestamp - min_timestamp) / (max_timestamp - min_timestamp);
                   });
    return timestamps_normalized;
}

inline auto ExtractTimestampsFromMsg(const PointCloud2::ConstSharedPtr msg,
                                     const PointField &timestamp_field) {
    auto extract_timestamps =
        [&msg]<typename T>(sensor_msgs::PointCloud2ConstIterator<T> &&it) -> std::vector<double> {
        const size_t n_points = msg->height * msg->width;
        std::vector<double> timestamps;
        timestamps.reserve(n_points);
        for (size_t i = 0; i < n_points; ++i, ++it) {
            timestamps.emplace_back(static_cast<double>(*it));
        }
        return NormalizeTimestamps(timestamps);
    };

    // According to the type of the timestamp == type, return a PointCloud2ConstIterator<type>
    using sensor_msgs::PointCloud2ConstIterator;
    if (timestamp_field.datatype == PointField::UINT32) {
        return extract_timestamps(PointCloud2ConstIterator<uint32_t>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT32) {
        return extract_timestamps(PointCloud2ConstIterator<float>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT64) {
        return extract_timestamps(PointCloud2ConstIterator<double>(*msg, timestamp_field.name));
    }

    // timestamp type not supported, please open an issue :)
    throw std::runtime_error("timestamp field type not supported");
}

inline std::unique_ptr<PointCloud2> CreatePointCloud2Msg(const size_t n_points,
                                                         const Header &header,
                                                         bool timestamp = false) {
    auto cloud_msg = std::make_unique<PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    cloud_msg->header = header;
    cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
    cloud_msg->fields.clear();
    int offset = 0;
    offset = addPointField(*cloud_msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "z", 1, PointField::FLOAT32, offset);
    offset += sizeOfPointField(PointField::FLOAT32);
    if (timestamp) {
        // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
        offset = addPointField(*cloud_msg, "time", 1, PointField::FLOAT64, offset);
        offset += sizeOfPointField(PointField::FLOAT64);
    }

    // Resize the point cloud accordingly
    cloud_msg->point_step = offset;
    cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
    cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

inline void FillPointCloud2XYZ(const std::vector<Eigen::Vector3d> &points, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
        const Eigen::Vector3d &point = points[i];
        *msg_x = point.x();
        *msg_y = point.y();
        *msg_z = point.z();
    }
}

inline void FillPointCloud2Timestamp(const std::vector<double> &timestamps, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<double> msg_t(msg, "time");
    for (size_t i = 0; i < timestamps.size(); i++, ++msg_t) *msg_t = timestamps[i];
}

inline std::vector<double> GetTimestamps(const PointCloud2::ConstSharedPtr msg) {
    auto timestamp_field = GetTimestampField(msg);
    if (!timestamp_field.has_value()) return {};

    // Extract timestamps from cloud_msg
    std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field.value());

    return timestamps;
}

inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2::ConstSharedPtr msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
    for (size_t i = 0; i < msg->height * msg->width; ++i, ++msg_x, ++msg_y, ++msg_z) {
        points.emplace_back(*msg_x, *msg_y, *msg_z);
    }
    return points;
}

inline std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                       const Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header);
    FillPointCloud2XYZ(points, *msg);
    return msg;
}

inline std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                       const Sophus::SE3d &T,
                                                       const Header &header) {
    std::vector<Eigen::Vector3d> points_t;
    points_t.resize(points.size());
    std::transform(points.cbegin(), points.cend(), points_t.begin(),
                   [&](const auto &point) { return T * point; });
    return EigenToPointCloud2(points_t, header);
}

inline std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                       const std::vector<double> &timestamps,
                                                       const Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header, true);
    FillPointCloud2XYZ(points, *msg);
    FillPointCloud2Timestamp(timestamps, *msg);
    return msg;
}

inline double quaternionToEulerAngles(double w, double x, double y, double z) {
    // // Roll (x-axis rotation)
    // float sinr_cosp = 2 * (w * x + y * z);
    // float cosr_cosp = 1 - 2 * (x * x + y * y);
    // roll = std::atan2(sinr_cosp, cosr_cosp);

    // // Pitch (y-axis rotation)
    // float sinp = 2 * (w * y - z * x);
    // if (std::abs(sinp) >= 1)
    //     pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    // else
    //     pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

template<typename T>
inline double stamp2Sec(const T& stamp) {
    return rclcpp::Time(stamp).seconds();
}

void LLtoUTM(const double Lat, const double Long,
             double &UTMNorthing, double &UTMEasting,
             char* UTMZone) {
    const double RADIANS_PER_DEGREE = M_PI / 180.0;
    const double DEGREES_PER_RADIAN = 180.0 / M_PI;

    // WGS84 Parameters
    const double WGS84_A = 6378137.0;		// major axis
    const double WGS84_B = 6356752.31424518;	// minor axis
    const double WGS84_F = 0.0033528107;		// ellipsoid flattening
    const double WGS84_E = 0.0818191908;		// first eccentricity
    const double WGS84_EP = 0.0820944379;		// second eccentricity

    // UTM Parameters
    const double UTM_K0 = 0.9996;			// scale factor
    const double UTM_FE = 500000.0;		// false easting
    const double UTM_FN_N = 0.0;			// false northing on north hemisphere
    const double UTM_FN_S = 10000000.0;		// false northing on south hemisphere
    const double UTM_E2 = (WGS84_E*WGS84_E);	// e^2
    const double UTM_E4 = (UTM_E2*UTM_E2);		// e^4
    const double UTM_E6 = (UTM_E4*UTM_E2);		// e^6
    const double UTM_EP2 = (UTM_E2/(1-UTM_E2));	// e'^2

    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double k0 = UTM_K0;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    //Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = (Long+180)-int((Long+180)/360)*360-180;

    double LatRad = Lat * RADIANS_PER_DEGREE;
    double LongRad = LongTemp * RADIANS_PER_DEGREE;
    double LongOriginRad;
    int    ZoneNumber;

    ZoneNumber = int((LongTemp + 180)/6) + 1;

    if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
        ZoneNumber = 32;

    // Special zones for Svalbard
    if( Lat >= 72.0 && Lat < 84.0 )
    {
        if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
        else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
        else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
        else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
    }
    
    // +3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

    //compute the UTM Zone from the latitude and longitude
    // snprintf(UTMZone, 4, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));

    eccPrimeSquared = (eccSquared)/(1-eccSquared);

    N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
    T = tan(LatRad)*tan(LatRad);
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
    A = cos(LatRad)*(LongRad-LongOriginRad);

    M = a*((1	- eccSquared/4		- 3*eccSquared*eccSquared/64	- 5*eccSquared*eccSquared*eccSquared/256)*LatRad
                - (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
                                    + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
                                    - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

    UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
                    + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
                    + 500000.0);

    UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
    if(Lat < 0)
        UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}



void LLtoUTM(const double Lat, const double Long,
             double &UTMNorthing, double &UTMEasting,
             std::string &UTMZone) {
    char zone_buf[] = {0, 0, 0, 0};

    LLtoUTM(Lat, Long, UTMNorthing, UTMEasting, zone_buf);

    UTMZone = zone_buf;
}
}  // namespace kiss_icp_ros::utils
