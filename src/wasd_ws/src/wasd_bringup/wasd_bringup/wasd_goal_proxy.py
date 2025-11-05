#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class WasdGoalProxy(Node):
    def __init__(self):
        super().__init__('wasd_goal_proxy')

        # 현재 로봇 위치 저장용
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_cb,
            10
        )

        # RViz Publish Point → 목표 입력
        self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_cb,
            10
        )

        # Nav2 액션 클라이언트 (기본 / 경유 두 종류)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.get_logger().info('wasd_goal_proxy node started. Click on map with "Publish Point" tool.')

        # ★★★ 여기 U자 경로 3포인트 좌표를 네 맵에 맞게 넣어줘 ★★★
        self.P_START = (3.29, 2.47)  # (x, y)
        self.P_MID   = (0.58, 2.61)
        self.P_END   = (3.13, 3.89)

        # ★★★ 여기 "U자 반대편 목표 영역" 범위도 네 맵에 맞게 조정 ★★★
        self.U_DEST_X_MIN = 2.8
        self.U_DEST_X_MAX = 3.45
        self.U_DEST_Y_MIN = 3.5
        self.U_DEST_Y_MAX = 4.25

    # 현재 로봇 위치 콜백
    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    # RViz에서 클릭했을 때
    def clicked_point_cb(self, msg: PointStamped):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(f'clicked_point frame_id is {msg.header.frame_id}, expected "map"')
        goal_x = msg.point.x
        goal_y = msg.point.y
        self.get_logger().info(f'Received clicked goal: x={goal_x:.2f}, y={goal_y:.2f}')

        # 로봇 현재 방향이 있으면 goal 방향 바라보게 yaw 계산
        if self.current_pose is not None:
            rx = self.current_pose.position.x
            ry = self.current_pose.position.y
            yaw = math.atan2(goal_y - ry, goal_x - rx)
        else:
            yaw = 0.0

        final_pose = self.make_pose(goal_x, goal_y, yaw)

        if self.is_u_turn_destination(goal_x, goal_y):
            self.get_logger().info('Goal is in U-turn zone → using [start, mid, end, goal] via NavigateThroughPoses')
            poses = [
                self.make_pose(self.P_START[0], self.P_START[1], yaw),  # yaw는 대충 같은 방향으로
                self.make_pose(self.P_MID[0],   self.P_MID[1],   yaw),
                self.make_pose(self.P_END[0],   self.P_END[1],   yaw),
                final_pose,
            ]
            self.send_nav_through_poses(poses)
        else:
            self.get_logger().info('Goal is normal → using NavigateToPose directly')
            self.send_nav_to_pose(final_pose)

    def is_u_turn_destination(self, x, y):
        return (self.U_DEST_X_MIN <= x <= self.U_DEST_X_MAX
                and self.U_DEST_Y_MIN <= y <= self.U_DEST_Y_MAX)

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_nav_to_pose(self, pose: PoseStamped):
        # 액션 서버 대기
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info('Sending NavigateToPose goal...')
        send_future = self.nav_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self.nav_to_pose_feedback_cb
        )
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

    def nav_to_pose_feedback_cb(self, feedback_msg):
        # 필요하면 로그 찍거나 관제 UI에 쓸 수 있음
        pass

    def send_nav_through_poses(self, poses):
        if not self.nav_through_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_through_poses action server not available')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ''

        self.get_logger().info(f'Sending NavigateThroughPoses goal with {len(poses)} poses...')
        send_future = self.nav_through_client.send_goal_async(
            goal,
            feedback_callback=self.nav_through_feedback_cb
        )
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

    def nav_through_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Current waypoint index: {fb.current_waypoint}')


def main(args=None):
    rclpy.init(args=args)
    node = WasdGoalProxy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()