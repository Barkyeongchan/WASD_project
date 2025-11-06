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

    # ✅ 1) 맵 파일: 네가 말한 절대 경로 사용
    map_file = LaunchConfiguration(
        'map',
        default='/home/wasd/WASD_project/src/wasd_ws/wasd_map.yaml'   # <- 여기 이름만 실제 이름으로
    )

    # ✅ 2) 네비 파라미터 파일: wasd_bringup/param/burger.yaml 사용
    #    (install/share/wasd_bringup/param/burger.yaml 을 가정)
    wasd_share_dir = get_package_share_directory('wasd_bringup')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(wasd_share_dir, 'param', 'burger.yaml')
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ✅ nav2_bringup 에 우리가 지정한 map / params_file 넘겨줌
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            package='wasd_bringup',
            executable='wasd_goal_proxy',
            name='wasd_goal_proxy',
            output='screen',
            emulate_tty=True,
        ),
    ])