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
    #pkg_robot_state_controller = get_package_share_directory('robot_state_controller')
    #pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    # Config
    #joy_config = os.path.join(pkg_kohm_gazebo, 'config/joystick',
    #                          'xbone.config.yaml')

    # Launch arguments
    drive_mode_switch_button = LaunchConfiguration('drive_mode_switch_button', default='8')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    follow_waypoints = LaunchConfiguration('follow_waypoints', default='false')
    
    
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
        ]), )
        
        
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
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'drive_mode_switch_button',
            default_value='8',
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

        # Nodes
        state_publishers,
        ign_gazebo,
        #joy_with_teleop_twist,
        #lidar_processor,
        
        #pointcloud_to_laserscan,
        #navigation,
        rviz,
        #waypoint_publisher,
        #robot_state_controller,
    ])
