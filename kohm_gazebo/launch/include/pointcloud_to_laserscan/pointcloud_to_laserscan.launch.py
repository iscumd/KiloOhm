# MIT License
#
# Copyright (c) 2021 Intelligent Systems Club
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_kohm_gazebo = get_package_share_directory('kohm_gazebo')

    # Config
    laserscan_config = os.path.join(pkg_kohm_gazebo,
                                    'config/pointcloud_to_laserscan',
                                    'pointcloud_to_laserscan.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes
    pointcloud_to_laserscan = Node(package='pointcloud_to_laserscan',
                                   executable='pointcloud_to_laserscan_node',
                                   remappings=[('cloud_in',
                                                '/kohm/combined_points'),
                                               ('scan', '/scan')],
                                   parameters=[{
                                       'laserscan_config': laserscan_config,
                                       'use_sim_time': use_sim_time,
                                   }],
                                   name='pointcloud_to_laserscan')

    laserscan_to_pointcloud = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud',
        remappings=[('scan_in', '/sick/scan'),
                    ('cloud', '/kohm/filtered_points')]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation time if true'),
        # Nodes
        pointcloud_to_laserscan,
        laserscan_to_pointcloud
    ])
