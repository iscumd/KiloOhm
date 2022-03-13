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
from nav2_common.launch import RewrittenYaml

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_kohm_gazebo = get_package_share_directory('kohm_robot')

    # Config
    vn300_conf = os.path.join(pkg_kohm_gazebo,
                              'config/vectornav',
                              'vectornav.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    param_substitutions = {
        'use_sim_time': use_sim_time,
    } #TODO add usb port to these params later

    configured_params = RewrittenYaml(source_file=vn300_conf,
                                      root_key='',
                                      param_rewrites=param_substitutions,
                                      convert_types=True)

    # Vectornav
    start_vectornav_cmd = Node(
        package='vectornav',
        executable='vectornav',
        output='screen',
        parameters=[configured_params])
    
    # Node that converts raw vectornav data to ros msgs
    start_vectornav_sensor_msgs_cmd = Node( 
        package='vectornav',
        executable='vn_sensor_msgs',
        output='screen',
        remappings=[('/vectornav/imu', '/kohm/gps/imu')],
        parameters=[configured_params])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation time if true'),
        # Nodes
        start_vectornav_cmd,
        start_vectornav_sensor_msgs_cmd
    ])
