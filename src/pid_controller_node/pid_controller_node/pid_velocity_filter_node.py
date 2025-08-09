#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class PIDVelocityFilterNode(Node):
    def __init__(self):
        super().__init__('pid_velocity_filter_node')

        # PID 파라미터
        self.declare_parameter("kp", 0.1)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("output_limit", 0.3)

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value
        self.output_limit = self.get_parameter("output_limit").value

        # 구독자
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )

        # 발행자
        self.publisher = self.create_publisher(
            Twist,
            '/filtered_cmd_vel',
            10
        )

        # 내부 상태
        self.last_error = 0.0
        self.integral = 0.0
        self.actual_velocity = 0.0

        self.prev_position = None
        self.prev_time = self.get_clock().now()

        self.get_logger().info("✅ PIDVelocityFilterNode started (using /tracked_pose for velocity)")

    def pose_callback(self, msg):
        now = self.get_clock().now()
        pos = msg.pose.position

        if self.prev_position is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 0:
                dx = pos.x - self.prev_position.x
                dy = pos.y - self.prev_position.y
                velocity = math.sqrt(dx**2 + dy**2) / dt
                self.actual_velocity = velocity

                self.get_logger().debug(
                    f"[POSE] dt={dt:.3f}, dx={dx:.3f}, dy={dy:.3f}, velocity={velocity:.3f}"
                )

        self.prev_position = pos
        self.prev_time = now

    def cmd_vel_callback(self, msg):
        target = msg.linear.x
        actual = self.actual_velocity
        error = target - actual

        self.integral += error
        self.integral = max(min(self.integral, 10.0), -10.0)  # anti-windup

        derivative = error - self.last_error
        self.last_error = error

        # PID 연산
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.output_limit), -self.output_limit)

        self.get_logger().info(
            f"[PID] Target={target:.3f}, Actual={actual:.3f}, Error={error:.3f}, Output={output:.3f}"
        )

        filtered = Twist()
        filtered.linear.x = output
        filtered.linear.y = msg.linear.y
        filtered.linear.z = msg.linear.z
        filtered.angular = msg.angular

        self.publisher.publish(filtered)

def main(args=None):
    rclpy.init(args=args)
    node = PIDVelocityFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
