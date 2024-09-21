#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot_ros = os.path.join('/home/abhishek/workspaces/line_follower_ws/src/turtlebot3_gazebo', 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('y_pose', default='-0.01')
    
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='-1.57')
    
    
    world = os.path.join(
        get_package_share_directory('follower'),
        'world',
        'maze_world.world'
    )
    
    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch')
    )

    # lidar_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
    #         launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items())
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot_ros, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot_ros, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }.items()
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link1_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(static_transform_publisher_node)

    return ld
