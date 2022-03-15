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
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_kohm_gazebo = get_package_share_directory('kohm_robot')

    white_lines_params_file_path = os.path.join(pkg_kohm_gazebo,
                                         'config/white_line_detection',
                                         'white_line_params.yaml')
    white_line_params_file = LaunchConfiguration('white_line_params_file',
                                           default=white_lines_params_file_path)
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    white_line_detection = Node(
        package='white_line_detection',
        executable='white_line_detection',
        name='white_line_detection',
        output='screen',
        remappings=[
            ('/camera/camera_points', '/kohm/camera_points'),
            ('/camera/image_raw', '/kohm/image_raw'),
            ('/camera/camera_info', '/kohm/camera_info')
        ],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'params_file': white_line_params_file}
        ])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('white_line_params_file',
                              default_value=white_lines_params_file_path,
                              description='The file path of the params file for White Line Detection'),
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),

        # Nodes
        white_line_detection,
    ])
