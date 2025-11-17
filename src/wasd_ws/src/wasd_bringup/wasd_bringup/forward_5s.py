#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Forward5s(Node):
    def __init__(self):
        super().__init__('forward_5s')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('🚀 5초간 전진 시작!')
        self.forward_speed = 0.2   # m/s
        self.duration = 5.0        # seconds
        self.rate_hz = 10

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        msg = Twist()

        if elapsed < self.duration:
            # 전진 중
            msg.linear.x = self.forward_speed
            self.pub.publish(msg)
        else:
            # 멈춤 명령 1회 보내고 종료
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)

            self.get_logger().info('🛑 5초 경과 → 정지 완료')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Forward5s()
    rclpy.spin(node)

if __name__ == '__main__':
    main()