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

        # ROS 2 파라미터 선언 및 기본값 설정
        self.declare_parameter('k_lin', 0.5)
        self.declare_parameter('k_ang', 0.2)
        self.declare_parameter('min_linear_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.8)
        self.declare_parameter('angle_tolerance_deg', 16.0)
        self.declare_parameter('pos_tolerance', 0.03)

        # 파라미터 값 가져오기
        self.update_parameters()

        # 파라미터 변경 콜백 설정
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.goal_sub = self.create_subscription(PoseStamped, '/goalpose1', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_1', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.01, self.control_loop)

        # 초기 파라미터 값 로깅
        self.log_parameters()

    def update_parameters(self):
        """파라미터 값들을 업데이트"""
        self.k_lin = self.get_parameter('k_lin').get_parameter_value().double_value
        self.k_ang = self.get_parameter('k_ang').get_parameter_value().double_value
        self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.min_angular_speed = self.get_parameter('min_angular_speed').get_parameter_value().double_value
        angle_tolerance_deg = self.get_parameter('angle_tolerance_deg').get_parameter_value().double_value
        self.angle_tolerance = math.radians(angle_tolerance_deg)
        self.pos_tolerance = self.get_parameter('pos_tolerance').get_parameter_value().double_value

    def parameter_callback(self, params):
        """파라미터 변경 시 호출되는 콜백"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name in ['k_lin', 'k_ang', 'min_linear_speed', 'min_angular_speed', 
                             'angle_tolerance_deg', 'pos_tolerance']:
                self.get_logger().info(f'파라미터 업데이트: {param.name} = {param.value}')
        
        # 파라미터 값 업데이트
        self.update_parameters()
        self.log_parameters()
        
        return SetParametersResult(successful=True)

    def log_parameters(self):
        """현재 파라미터 값들을 로깅"""
        self.get_logger().info(
            f"제어 파라미터 - k_lin: {self.k_lin:.3f}, k_ang: {self.k_ang:.3f}, "
            f"min_linear: {self.min_linear_speed:.3f}, min_angular: {self.min_angular_speed:.3f}, "
            f"angle_tol: {math.degrees(self.angle_tolerance):.1f}°, pos_tol: {self.pos_tolerance:.3f}m"
        )

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

        if abs(yaw_error) > self.angle_tolerance:
            cmd.angular.z = self.k_ang * yaw_error
            # 최소 각속도 보장
            if abs(cmd.angular.z) < self.min_angular_speed:
                cmd.angular.z = math.copysign(self.min_angular_speed, cmd.angular.z)
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.k_lin * distance
            # 최소 선속도 보장
            if abs(cmd.linear.x) < self.min_linear_speed:
                cmd.linear.x = math.copysign(self.min_linear_speed, cmd.linear.x)
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