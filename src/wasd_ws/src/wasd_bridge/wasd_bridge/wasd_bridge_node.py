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

        # 로봇 -> 관제 UI 도착 신호
        self.nav_pub = self.create_publisher(
            String,
            '/nav',
            10
        )

        # Nav2 액션 클라이언트
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # points.yaml 기본 경로
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

    # UI -> 명령 수신 콜백
    def ui_command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Received UI command: {cmd}')

        # 허용되는 명령만 처리
        if cmd not in ['START', 'RACK_A', 'RACK_B', 'WAIT']:
            self.get_logger().warn(f'Unknown command: {cmd}')
            return

        # 목적지 pose 만들기
        goal_pose = self.make_pose_from_key(cmd)

        # 바로 Nav2로 목표 전송
        self.get_logger().info(f'Navigating directly to: {cmd}')
        self.send_nav_to_pose(goal_pose, cmd)
            
        # 포인트 -> PoseStamped    
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
    
    # 도착 신호 publish
    def publish_arrived(self, target_name: str):
        """
        관제 UI 요구 사항:
            - RACK_A 도착 -> "ARRIVED:RACK_A"
            - RACK_B 도착 -> "ARRIVED:RACK_B"
            - WAIT   도착 -> "ARRIVED:WAIT"
        START는 신호 안 보냄
        """
        if target_name not in ['RACK_A', 'RACK_B', 'WAIT']:
            # 관제에서 안 쓰는 타깃이면 패스
            self.get_logger().info(f'Arrived at {target_name}, but no ARRIVED message needed.')
            return

        msg = String()
        msg.data = f'ARRIVED:{target_name}'
        self.get_logger().info(f'[NAV] published: {msg.data}')
        self.nav_pub.publish(msg)

    # Nav2 액션 호출 부분
    def send_nav_to_pose(self, pose: PoseStamped, target_name: str):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f'Sending NavigateToPose goal to {target_name}...')

        # 액션 서버로 goal 전송, 콜백에 target_name 같이 넘겨줌
        send_future = self.nav_to_pose_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, target=target_name: self.nav_to_pose_response_cb(fut, target)
        )

    def nav_to_pose_response_cb(self, future, target_name: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'NavigateToPose goal rejected for {target_name}')
            return

        self.get_logger().info(f'NavigateToPose goal accepted for {target_name}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, target=target_name: self.nav_to_pose_result_cb(fut, target)
        )

    def nav_to_pose_result_cb(self, future, target_name: str):
        result = future.result().result
        self.get_logger().info(f'NavigateToPose finished for {target_name} with result: {result}')

        # 도착했다고 가정하고 관제에 도착 신호 전송
        self.publish_arrived(target_name)

    # (지금은 안 쓰지만 남겨둔 함수 – 나중에 경유지 여러 개 쓸 때 재활용 가능)
    def send_nav_through_poses(self, poses, target_name: str = ""):
        if not self.nav_through_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_through_poses action server not available')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ''

        self.get_logger().info(
            f'Sending NavigateThroughPoses goal with {len(poses)} poses... '
            f'target={target_name}'
        )

        send_future = self.nav_through_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut, target=target_name: self.nav_through_response_cb(fut, target)
        )

    def nav_through_response_cb(self, future, target_name: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'NavigateThroughPoses goal rejected for {target_name}')
            return

        self.get_logger().info(f'NavigateThroughPoses goal accepted for {target_name}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, target=target_name: self.nav_through_result_cb(fut, target)
        )

    def nav_through_result_cb(self, future, target_name: str):
        result = future.result().result
        self.get_logger().info(
            f'NavigateThroughPoses finished for {target_name} with result: {result}'
        )
        self.publish_arrived(target_name)


def main(args=None):
    rclpy.init(args=args)
    node = WasdBridge()
    rclpy.spin(node)
    rclpy.shutdown()


def cli_main():
    main()


if __name__ == '__main__':
    main()