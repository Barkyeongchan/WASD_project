#!/usr/bin/env python3
import math
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

import os
from ament_index_python.packages import get_package_share_directory

def yaw_to_quat(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class WasdBridge(Node):
    def __init__(self):
        super().__init__('wasd_bridge')

        # 관제 UI에서 오는 명령
        self.create_subscription(
            String,
            '/wasd_ui_command',
            self.ui_command_cb,
            10
        )

        # Nav2 액션 클라이언트
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        default_points = os.path.join(
            get_package_share_directory('wasd_bridge'),
            'config',
            'points.yaml'
        )
        # 포인트 설정 로드
        points_path = Path(self.declare_parameter(
            'points_yaml', default_points).get_parameter_value().string_value)

        with open(points_path, 'r') as f:
            self.points = yaml.safe_load(f)

        self.get_logger().info(f'Loaded points from: {points_path}')
        self.get_logger().info('wasd_bridge node started. Waiting for /wasd_ui_command...')

    def ui_command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Received UI command: {cmd}')

        if cmd not in ['START', 'RACK_A', 'RACK_B']:
            self.get_logger().warn(f'Unknown command: {cmd}')
            return

        # 목적지 pose 만들기
        goal_pose = self.make_pose_from_key(cmd)

        # --------- U코너 로직 (위치 사용 X 버전) ---------
        # START <-> RACK_A 이동일 때는 항상 mid_a를 경유하도록 설계
        if cmd == 'RACK_A':
            # START에서 RACK_A로 가는 명령이라고 가정
            mid_pose = self.make_pose_from_key('mid_a')
            self.get_logger().info('Using mid_a -> RACK_A via NavigateThroughPoses')
            self.send_nav_through_poses([mid_pose, goal_pose])

        elif cmd == 'START':
            # RACK_A에서 START로 가는 명령이라고 가정
            mid_pose = self.make_pose_from_key('mid_a')
            self.get_logger().info('Using mid_a -> START via NavigateThroughPoses')
            self.send_nav_through_poses([mid_pose, goal_pose])

        else:
            # 그 외에는 그냥 바로 NavigateToPose (예: RACK_B 등)
            self.get_logger().info('Using direct NavigateToPose')
            self.send_nav_to_pose(goal_pose)

    def make_pose_from_key(self, key):
        p = self.points[key]
        x = p['x']
        y = p['y']
        yaw = math.radians(p['yaw_deg'])
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # === Nav2 액션 호출 부분 ===
    def send_nav_to_pose(self, pose: PoseStamped):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info('Sending NavigateToPose goal...')
        send_future = self.nav_to_pose_client.send_goal_async(goal)
        send_future.add_done_callback(self.nav_to_pose_response_cb)

    def nav_to_pose_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            return
        self.get_logger().info('NavigateToPose goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_to_pose_result_cb)

    def nav_to_pose_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'NavigateToPose finished with result: {result}')

    def send_nav_through_poses(self, poses):
        if not self.nav_through_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_through_poses action server not available')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ''

        self.get_logger().info(f'Sending NavigateThroughPoses goal with {len(poses)} poses...')
        send_future = self.nav_through_client.send_goal_async(goal)
        send_future.add_done_callback(self.nav_through_response_cb)

    def nav_through_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateThroughPoses goal rejected')
            return
        self.get_logger().info('NavigateThroughPoses goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_through_result_cb)

    def nav_through_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'NavigateThroughPoses finished with result: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = WasdBridge()
    rclpy.spin(node)
    rclpy.shutdown()

def cli_main():
    main()

if __name__ == '__main__':
    main()

