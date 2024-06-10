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
from ament_index_python.packages import get_package_share_directory
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
    share_dir = get_package_share_directory('kiss_icp')

    parameter_file = os.path.join(
            share_dir, 'config', 'params.yaml')

    return LaunchDescription(
        [
            Node(
                package="kiss_icp",
                executable="odometry_node",
                name="odometry_node",
                output="screen",
                parameters=[parameter_file],
            ),
            Node(
                package="kiss_icp",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[parameter_file],
            ),
            Node(
                package="kiss_icp",
                executable="ins_node",
                name="ins_node",
                output="screen",
                parameters=[parameter_file],
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
