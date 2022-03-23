import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ROS packages
    pkg_kohm_gazebo = get_package_share_directory('kohm_gazebo')
    pkg_robot_state_controller = get_package_share_directory(
        'robot_state_controller')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    # Config
    joy_config = os.path.join(pkg_kohm_gazebo, 'config/joystick',
                              'wii-wheel.config.yaml')

    # Launch arguments
    drive_mode_switch_button = LaunchConfiguration(
        'drive_mode_switch_button', default='7')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    follow_waypoints = LaunchConfiguration('follow_waypoints', default='false')
    gazebo_world = LaunchConfiguration(
        'gazebo_world', default='kohms_world_shapes.sdf')

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/state_publishers/state_publishers.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/gazebo/gazebo.launch.py'
        ]),
        launch_arguments={
            'gazebo_world': gazebo_world
        }.items()
    )

    joy_with_teleop_twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop_twist_joy, 'launch', 'teleop-launch.py')),
        launch_arguments={
            'joy_dev': '/dev/input/js0',
            'config_filepath': joy_config
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/rviz/rviz.launch.py'
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
        }.items(),
    )

    lidar_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/lidar_processor/lidar_processor.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    sensor_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/sensor_processor/sensor_processor.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    pointcloud_to_laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/pointcloud_to_laserscan/pointcloud_to_laserscan.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/navigation/navigation.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo, 'launch'),
            '/include/waypoint_publisher/waypoint.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'follow_waypoints': follow_waypoints
        }.items(),
    )

    robot_state_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_robot_state_controller, 'launch'),
            '/rsc_with_ipp.launch.py'
        ]),
        launch_arguments={
            'switch_button': drive_mode_switch_button,
            'use_sim_time': use_sim_time
        }.items(),
    )

    white_line_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_kohm_gazebo,
                         'launch/include/white_line_detection'),
            '/white_line_detection.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'drive_mode_switch_button',
            default_value='10',
            description='Which button is used on the joystick to switch drive mode. (In joy message)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_rviz',
                              default_value='true',
                              description='Open rviz if true'),
        DeclareLaunchArgument('follow_waypoints',
                              default_value='false',
                              description='follow way points if true'),
        DeclareLaunchArgument('gazebo_world',
                              default_value='kohms_world_shapes.sdf',
                              description='gazebo world to load'),

        # Nodes
        state_publishers,
        ign_gazebo,
        joy_with_teleop_twist,
        lidar_processor,

        sensor_processor,
        pointcloud_to_laserscan,
        navigation,
        rviz,

        waypoint_publisher,
        robot_state_controller,
        white_line_detection
    ])
