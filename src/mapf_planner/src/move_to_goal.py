#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class GoalMover(Node):
    def __init__(self):
        super().__init__('goal_mover')
        self.goal_pose = None
        self.current_pose = None
        self.yaw = 0.0

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_1', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Control parameters
        self.k_lin = 0.5
        self.k_ang = 0.3
        self.angle_tolerance = math.radians(16)  
        self.pos_tolerance = 0.03  # 5 cm

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f'Received new goal: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.goal_pose is None or self.current_pose is None:
            return

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - self.yaw)

        # 👉 현재 위치와 목표 위치 출력
        self.get_logger().info(
            f"현재 위치: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}) | "
            f"목표 위치: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})"
        )

        cmd = Twist()

        if distance < self.pos_tolerance:
            self.get_logger().info("✅ 목표 위치 도달. 정지합니다.")
            self.goal_pose = None
            self.cmd_pub.publish(Twist())
            return

        # 최소 속도 설정
        min_linear_speed = 0.55
        min_angular_speed = 0.55

        if abs(yaw_error) > self.angle_tolerance:
            cmd.angular.z = self.k_ang * yaw_error
            # 최소 각속도 보장
            if abs(cmd.angular.z) < min_angular_speed:
                cmd.angular.z = math.copysign(min_angular_speed, cmd.angular.z)
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.k_lin * distance
            # 최소 선속도 보장
            if abs(cmd.linear.x) < min_linear_speed:
                cmd.linear.x = math.copysign(min_linear_speed, cmd.linear.x)
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
