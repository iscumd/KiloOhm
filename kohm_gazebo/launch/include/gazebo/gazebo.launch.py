import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch.substitutions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_kohm_description = get_package_share_directory(
        'kohm_description')
    pkg_kohm_gazebo = get_package_share_directory('kohm_gazebo')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='kohms_world_shapes.sdf')

    # Nodes
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch',
                         'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': launch.substitutions.PathJoinSubstitution([pkg_kohm_gazebo + '/worlds/', gazebo_world])
        }.items()
    )

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/kohms_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/kohm/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/kohm/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/kohm/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/kohm/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        output='screen',
        remappings=[
            ('/world/kohms_world/clock', '/clock'),
            ('/model/kohm/tf', '/tf'),
            ('/model/kohm/cmd_vel', '/robot/cmd_vel'),
            ('/model/kohm/odometry', '/kohm/odom'),
            ('/lidar', '/lidar/unfiltered_scan'),
            ('/lidar/points', '/kohm/raw_points'),
            ('/imu', '/kohm/imu'),
            ('/rgbd_camera/image', '/kohm/image_raw'),
            ('/rgbd_camera/depth_image', '/kohm/depth_image_raw'),
            ('/rgbd_camera/camera_info', '/kohm/camera_info'),
            ('/rgbd_camera/points', '/kohm/camera_depth_points'),
            ('/model/kohm/joint_state', 'joint_states')
        ])

    #('/model/kohm/joint_state', 'joint_states'),
    ign_spawn_robot = Node(package='ros_ign_gazebo',
                           executable='create',
                           arguments=[
                               '-name', 'kohm', '-x', '0', '-z', '0', '-Y',
                               '0', '-topic', 'robot_description'
                           ],
                           output='screen')

    return LaunchDescription([
        # Nodes
        ign_gazebo,
        ign_bridge,
        ign_spawn_robot,

        DeclareLaunchArgument('gazebo_world',
                              default_value='kohms_word_shapes.sdf',
                              description='gazebo world to load'),
    ])
