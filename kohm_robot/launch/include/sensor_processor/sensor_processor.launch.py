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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    sensor_processor = Node( #TODO pretty sure we can remove this for robot
        package='sensor_processor',
        executable='sensor_processor',
        name='sensor_processor',
        output='screen',
        remappings=[
            ('/camera/image_raw', '/kohm/image_raw'),
            ('/camera/unfiltered_image_raw', '/kohm/unfiltered_image_raw'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }])

    cloud_concat = Node(
        package='sensor_processor',
        executable='pointcloud',
        name='PointCloud_Concatenate',
        output='screen',
        remappings=[
            ('/lidar/points', '/kohm/filtered_points'),
            ('/camera/points', '/kohm/camera_points'),
            ('/combined/points', '/kohm/combined_points'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'),

        # Nodes
        sensor_processor,
        cloud_concat
    ])
