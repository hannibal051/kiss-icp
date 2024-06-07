# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    ThisLaunchFileDir
)
import os


def generate_launch_description():
    current_pkg = FindPackageShare("kiss_icp")

    # Use FindPackageShare to locate the launch file within another package
    included_ekf_file_path = PathJoinSubstitution([
        FindPackageShare('robot_localization'),
        'launch',
        'localization.launch.py'
    ])

    included_imu_file_path = PathJoinSubstitution([
        FindPackageShare('lio_sam'),
        'launch',
        'run.launch.py'
    ])

    return LaunchDescription(
        [
            # ROS 2 parameters
            DeclareLaunchArgument("pc_topic", default_value="/vehicle_6/luminar_id0_cloud"),
            DeclareLaunchArgument("gps_topic", default_value="/eskf/odom"),
            DeclareLaunchArgument("bagfile", default_value=""),
            DeclareLaunchArgument("visualize", default_value="false"),
            DeclareLaunchArgument("odom_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="luminar"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            # KISS-ICP parameters
            DeclareLaunchArgument("deskew", default_value="true"),
            DeclareLaunchArgument("max_range", default_value="100.0"),
            DeclareLaunchArgument("min_range", default_value="1.0"),
            # GNSS re-relocalization parameters
            DeclareLaunchArgument("echo_gps", default_value="true"),
            DeclareLaunchArgument("origin_set", default_value="false"),
            DeclareLaunchArgument("origin_lat", default_value="0.0"),
            DeclareLaunchArgument("origin_lon", default_value="0.0"),
            DeclareLaunchArgument("origin_alt", default_value="0.0"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(included_imu_file_path)
            ),
            Node(
                package="kiss_icp",
                executable="odometry_node",
                name="odometry_node",
                output="screen",
                remappings=[("pointcloud_topic", LaunchConfiguration("pc_topic")),
                            ("gps_filter_topic", LaunchConfiguration("gps_topic"))],
                parameters=[
                    {
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "max_range": LaunchConfiguration("max_range"),
                        "min_range": LaunchConfiguration("min_range"),
                        "deskew": LaunchConfiguration("deskew"),
                        "echo_gps": LaunchConfiguration("echo_gps"),
                        #  "voxel_size": LaunchConfiguration("voxel_size"),
                        "max_points_per_voxel": 20,
                        "initial_threshold": 2.0,
                        "min_motion_th": 0.1,
                        "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
                        "visualize": LaunchConfiguration("visualize"),
                    }
                ],
            ),
            Node(
                package="kiss_icp",
                executable="ins_node",
                name="ins_node",
                output="screen",
                parameters=[
                    {
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        # GNSS re-relocalization parameters
                        "origin_set": LaunchConfiguration("origin_set"),
                        "origin_lat": LaunchConfiguration("origin_lat"),
                        "origin_lon": LaunchConfiguration("origin_lon"),
                        "origin_alt": LaunchConfiguration("origin_alt"),
                    }
                ],
                # At least one /fix data (sensor_msgs/msg/nav_sat_fix)
                # More data more robust
                remappings=[("/imu/data", "/vehicle_6/novatel_top_id0_imu"), ## /vehicle_8/novatel_top/rawimux
                            ("/gps/front/fix", "/vehicle_6/novatel_top_id0_gps"),   ## /vehicle_8/novatel_top/fix
                            ("/gps/rear/fix", "/vehicle_6/novatel_btm_id0_gps"), ## /vehicle_8/novatel_bottom/fix
                            ("/ins/odometry", "/vehicle_6/local_odometry")], ## /vehicle_8/local_odometry
            ),
            #Node(
            #    package="rviz2",
            #    executable="rviz2",
            #    output={"both": "log"},
            #    arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "kiss_icp.rviz"])],
            #    condition=IfCondition(LaunchConfiguration("visualize")),
            #),
            # ExecuteProcess(
            #     cmd=["ros2", "bag", "play", LaunchConfiguration("bagfile")],
            #     output="screen",
            #     condition=IfCondition(
            #         PythonExpression(["'", LaunchConfiguration("bagfile"), "' != ''"])
            #     ),
            # ),
        ]
    )
