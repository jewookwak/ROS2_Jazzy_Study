#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import time

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class PIDController:
    """PID 제어기 클래스"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, min_output=-1.0, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
    def update_gains(self, kp, ki, kd):
        """PID 게인 업데이트"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def reset(self):
        """PID 상태 초기화"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
    def compute(self, error, current_time=None):
        """PID 출력 계산"""
        if current_time is None:
            current_time = time.time()
            
        if self.last_time is None:
            self.last_time = current_time
            self.previous_error = error
            return 0.0
            
        dt = current_time - self.last_time
        if dt <= 0.0:
            return 0.0
            
        # Proportional term
        proportional = self.kp * error
        
        # Integral term with windup protection
        self.integral += error * dt
        # Anti-windup: clamp integral to reasonable bounds
        max_integral = abs(self.max_output) / (self.ki + 1e-6)
        self.integral = max(-max_integral, min(max_integral, self.integral))
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Total output
        output = proportional + integral + derivative
        
        # Output saturation
        output = max(self.min_output, min(self.max_output, output))
        
        # Update for next iteration
        self.previous_error = error
        self.last_time = current_time
        
        return output

class GoalMover(Node):
    def __init__(self):
        super().__init__('goal_mover')
        self.goal_pose = None
        self.current_pose = None
        self.yaw = 0.0

        # ROS 2 파라미터 선언 및 기본값 설정
        # 기본 제어 게인
        self.declare_parameter('k_lin', 0.5)  # P 제어 게인 (하위 호환)
        self.declare_parameter('k_ang', 0.2)  # P 제어 게인 (하위 호환)
        
        # 선형 PID 파라미터
        self.declare_parameter('linear_P', 0.5)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        
        # 각속도 PID 파라미터
        self.declare_parameter('angular_P', 0.2)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.0)
        
        # 최소 속도 설정
        self.declare_parameter('min_linear_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.8)
        
        # 허용 오차
        self.declare_parameter('angle_tolerance_deg', 16.0)
        self.declare_parameter('pos_tolerance', 0.03)
        
        # PID 제어 활성화 플래그
        self.declare_parameter('enable_pid', 1.0)
        
        # 출력 제한
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 3.0)

        # 파라미터 값 가져오기
        self.update_parameters()

        # PID 컨트롤러 초기화
        self.linear_pid = PIDController(
            kp=self.linear_P, 
            ki=self.linear_I, 
            kd=self.linear_D,
            min_output=-self.max_linear_speed,
            max_output=self.max_linear_speed
        )
        
        self.angular_pid = PIDController(
            kp=self.angular_P, 
            ki=self.angular_I, 
            kd=self.angular_D,
            min_output=-self.max_angular_speed,
            max_output=self.max_angular_speed
        )

        # 파라미터 변경 콜백 설정
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 구독자와 퍼블리셔
        self.goal_sub = self.create_subscription(PoseStamped, '/goalpose1', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_1', self.odom_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 오차 정보 퍼블리셔 (모니터링용)
        self.distance_error_pub = self.create_publisher(Float64, '/distance_error', 10)
        self.angle_error_pub = self.create_publisher(Float64, '/angle_error', 10)

        # 제어 루프 타이머
        self.timer = self.create_timer(0.01, self.control_loop)

        # 초기 파라미터 값 로깅
        self.log_parameters()

    def update_parameters(self):
        """파라미터 값들을 업데이트"""
        # 기본 게인 (하위 호환)
        self.k_lin = self.get_parameter('k_lin').get_parameter_value().double_value
        self.k_ang = self.get_parameter('k_ang').get_parameter_value().double_value
        
        # PID 파라미터
        self.linear_P = self.get_parameter('linear_P').get_parameter_value().double_value
        self.linear_I = self.get_parameter('linear_I').get_parameter_value().double_value
        self.linear_D = self.get_parameter('linear_D').get_parameter_value().double_value
        
        self.angular_P = self.get_parameter('angular_P').get_parameter_value().double_value
        self.angular_I = self.get_parameter('angular_I').get_parameter_value().double_value
        self.angular_D = self.get_parameter('angular_D').get_parameter_value().double_value
        
        # 최소/최대 속도
        self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.min_angular_speed = self.get_parameter('min_angular_speed').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # 허용 오차
        angle_tolerance_deg = self.get_parameter('angle_tolerance_deg').get_parameter_value().double_value
        self.angle_tolerance = math.radians(angle_tolerance_deg)
        self.pos_tolerance = self.get_parameter('pos_tolerance').get_parameter_value().double_value
        
        # PID 활성화
        self.enable_pid = self.get_parameter('enable_pid').get_parameter_value().double_value > 0.5
        
        # k_lin, k_ang를 PID P 게인과 동기화 (하위 호환)
        if hasattr(self, 'linear_pid'):
            self.linear_pid.update_gains(self.linear_P, self.linear_I, self.linear_D)
            self.linear_pid.max_output = self.max_linear_speed
            self.linear_pid.min_output = -self.max_linear_speed
            
        if hasattr(self, 'angular_pid'):
            self.angular_pid.update_gains(self.angular_P, self.angular_I, self.angular_D)
            self.angular_pid.max_output = self.max_angular_speed
            self.angular_pid.min_output = -self.max_angular_speed

    def parameter_callback(self, params):
        """파라미터 변경 시 호출되는 콜백"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name in ['k_lin', 'k_ang', 'linear_P', 'linear_I', 'linear_D',
                             'angular_P', 'angular_I', 'angular_D',
                             'min_linear_speed', 'min_angular_speed', 
                             'max_linear_speed', 'max_angular_speed',
                             'angle_tolerance_deg', 'pos_tolerance', 'enable_pid']:
                self.get_logger().info(f'파라미터 업데이트: {param.name} = {param.value}')
        
        # 파라미터 값 업데이트
        self.update_parameters()
        self.log_parameters()
        
        return SetParametersResult(successful=True)

    def log_parameters(self):
        """현재 파라미터 값들을 로깅"""
        mode = "PID" if self.enable_pid else "P"
        self.get_logger().info(
            f"제어 모드: {mode} | "
            f"Linear PID: P={self.linear_P:.3f}, I={self.linear_I:.3f}, D={self.linear_D:.3f} | "
            f"Angular PID: P={self.angular_P:.3f}, I={self.angular_I:.3f}, D={self.angular_D:.3f}"
        )
        self.get_logger().info(
            f"속도 제한 - Linear: {self.min_linear_speed:.2f}~{self.max_linear_speed:.2f} m/s, "
            f"Angular: {self.min_angular_speed:.2f}~{self.max_angular_speed:.2f} rad/s"
        )
        self.get_logger().info(
            f"허용 오차 - 각도: {math.degrees(self.angle_tolerance):.1f}°, 위치: {self.pos_tolerance:.3f}m"
        )

    def goal_callback(self, msg):
        """목표 위치 수신 콜백"""
        self.goal_pose = msg.pose
        # 새로운 목표 수신 시 PID 상태 초기화
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.get_logger().info(
            f'새로운 목표 수신: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})'
        )

    def odom_callback(self, msg):
        """오도메트리 수신 콜백"""
        self.current_pose = msg.pose.pose

        # 쿼터니언에서 yaw 각도 추출
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """메인 제어 루프"""
        if self.goal_pose is None or self.current_pose is None:
            return

        # 현재 위치와 목표 위치 간의 거리 및 각도 계산
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - self.yaw)

        # 오차 정보 발행 (모니터링용)
        distance_msg = Float64()
        distance_msg.data = distance
        self.distance_error_pub.publish(distance_msg)
        
        angle_msg = Float64()
        angle_msg.data = yaw_error
        self.angle_error_pub.publish(angle_msg)

        # 현재 상태 로깅
        if self.get_clock().now().nanoseconds % 1000000000 < 10000000:  # 1초마다 로깅
            self.get_logger().info(
                f"현재: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, {math.degrees(self.yaw):.1f}°) | "
                f"목표: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f}) | "
                f"거리: {distance:.3f}m, 각도오차: {math.degrees(yaw_error):.1f}°"
            )

        cmd = Twist()

        # 목표 도달 확인
        if distance < self.pos_tolerance:
            self.get_logger().info("✅ 목표 위치 도달. 정지합니다.")
            self.goal_pose = None
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.cmd_pub.publish(Twist())  # 정지 명령
            return

        # 제어 로직: 각도 먼저 맞춘 후 직진
        if abs(yaw_error) > self.angle_tolerance:
            # 회전 제어
            if self.enable_pid:
                # PID 제어
                angular_output = self.angular_pid.compute(yaw_error)
            else:
                # P 제어 (기존 방식)
                angular_output = self.angular_P * yaw_error
                angular_output = max(-self.max_angular_speed, 
                                   min(self.max_angular_speed, angular_output))
            
            # 최소 각속도 보장
            if abs(angular_output) < self.min_angular_speed:
                angular_output = math.copysign(self.min_angular_speed, angular_output)
            
            cmd.angular.z = angular_output
            cmd.linear.x = 0.0
            
        else:
            # 직진 제어
            if self.enable_pid:
                # PID 제어
                linear_output = self.linear_pid.compute(distance)
            else:
                # P 제어 (기존 방식)
                linear_output = self.linear_P * distance
                linear_output = max(-self.max_linear_speed, 
                                  min(self.max_linear_speed, linear_output))
            
            # 최소 선속도 보장
            if abs(linear_output) < self.min_linear_speed:
                linear_output = math.copysign(self.min_linear_speed, linear_output)
            
            cmd.linear.x = linear_output
            cmd.angular.z = 0.0

        # 속도 명령 발행
        self.cmd_pub.publish(cmd)

        # 디버그 정보 (10Hz로 제한)
        current_time_ns = self.get_clock().now().nanoseconds
        if current_time_ns % 100000000 < 10000000:  # 0.1초마다
            mode = "PID" if self.enable_pid else "P"
            self.get_logger().debug(
                f"[{mode}] cmd: lin={cmd.linear.x:.3f}, ang={cmd.angular.z:.3f} | "
                f"distance={distance:.3f}, yaw_error={math.degrees(yaw_error):.1f}°"
            )

def main(args=None):
    rclpy.init(args=args)
    
    # 노드 생성 및 실행
    node = GoalMover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    except Exception as e:
        node.get_logger().error(f"오류 발생: {e}")
    finally:
        # 정리 작업
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()