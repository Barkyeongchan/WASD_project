#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wasd_bringup',        # 우리가 만든 패키지
            executable='teleop_keyboard',  # setup.py에 등록할 이름
            name='teleop_keyboard',
            output='screen',
        )
    ])