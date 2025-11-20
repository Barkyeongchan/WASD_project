#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 맵 절대 경로
    map_file = '/home/wasd/WASD_project/src/wasd_ws/wasd_map.yaml'

    # burger.yaml 절대 경로
    params_file = '/home/wasd/WASD_project/src/wasd_ws/src/wasd_bringup/param/burger.yaml'

    map_arg = LaunchConfiguration(
        'map',
        default=map_file
    )

    params_arg = LaunchConfiguration(
        'params_file',
        default=params_file
    )

    # nav2_bringup 의 bringup_launch.py
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    # RViz 설정은 기존 TB3 설정 사용
    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_arg,
            description='Full path to map file to load'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_arg,
            description='Full path to param file to load'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Nav2 bringup 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_arg,
                'use_sim_time': use_sim_time,
                'params_file': params_arg,
            }.items(),
        ),

        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # (선택) wasd_goal_proxy 계속 쓸 거면 남겨두고, 안 쓰면 주석 처리
        # Node(
        #     package='wasd_bringup',
        #     executable='wasd_goal_proxy',
        #     name='wasd_goal_proxy',
        #     output='screen',
        #     emulate_tty=True
        # ),
    ])