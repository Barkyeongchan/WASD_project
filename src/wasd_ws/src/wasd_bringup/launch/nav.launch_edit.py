#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ✅ 네가 실제 사용하는 맵 경로 (절대 경로로)
    map_dir = LaunchConfiguration(
        'map',
        default='/home/wasd/WASD_project/src/wasd_ws/wasd_map_clean.yaml'
    )

    # ✅ 네가 수정한 burger.yaml 경로
    param_dir = LaunchConfiguration(
        'params_file',
        default='/home/wasd/WASD_project/src/wasd_ws/src/wasd_bringup/param/burger.yaml'
    )

    # nav2_bringup 의 bringup_launch.py (Nav2 핵심 로직)
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # RViz 설정은 그대로 turtlebot3 navigation2 의 설정 사용
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    return LaunchDescription([
        # launch argument 등록
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ✅ bringup_launch.py 호출 (Nav2의 모든 서버들: map, planner, controller, costmap 등)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_file_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
            }.items(),
        ),

        # ✅ RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # ✅ wasd_goal_proxy 실행
        Node(
            package='wasd_bringup',
            executable='wasd_goal_proxy',
            name='wasd_goal_proxy',
            output='screen',
            emulate_tty=True
        ),
    ])