#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
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
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_output=1.0, min_output=-1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        
    def update(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return 0.0
            
        # PID 계산
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 출력 제한
        output = max(min(output, self.max_output), self.min_output)
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

class RobotGoalController(Node):
    def __init__(self):
        super().__init__('_robot_goal_controller')
        
        # 로봇 ID 파라미터 추가 (실행 시 설정 가능)
        self.declare_parameter('robot_id', 3)  # 기본값 3
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        
        # 상태 정의
        self.IDLE = 'IDLE'
        self.ROTATE_TO_GOAL = 'RotateToGoal'
        self.MOVE_TO_GOAL = 'MoveToGoal'
        self.ROTATE_TO_FINAL = 'RotateToFinal'
        self.GOAL_REACHED = 'GoalReached'
        
        self.current_state = self.IDLE
        self.goal_pose = None
        self.current_pose = None
        self.current_yaw = 0.0
        
        # PID 파라미터 (나중에 파라미터로 설정 가능)
        self.angle_tolerance = 0.1  # 라디안 (약 5.7도)
        self.distance_tolerance = 0.05  # 미터
        
        # PID 컨트롤러 초기화
        self.angular_pid = PIDController(
            kp=1.0, ki=0.0, kd=0.1, max_output=2.0, min_output=-2.0)
        self.linear_pid = PIDController(
            kp=0.5, ki=0.0, kd=0.1, max_output=1.0, min_output=-1.0)
        
        # 파라미터 선언
        try:
            self.declare_parameter('angular_kp', 1.0)
            self.declare_parameter('angular_ki', 0.0)
            self.declare_parameter('angular_kd', 0.1)
            self.declare_parameter('linear_kp', 0.5)
            self.declare_parameter('linear_ki', 0.0)
            self.declare_parameter('linear_kd', 0.1)
            self.declare_parameter('angle_tolerance', 0.1)
            self.declare_parameter('distance_tolerance', 0.05)
        except Exception as e:
            self.get_logger().warn(f"파라미터 선언 중 오류: {e}")
        
        # === 도메인 브릿지와 맞추기 위한 토픽 이름 수정 ===
        # 구독자들 - 도메인 브릿지 remap에 맞춤
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)  # remap될 예정
        self.odom_sub = self.create_subscription(
            Odometry, f'odom_{self.robot_id}', self.odom_callback, 10)
            
        # 발행자들 - 도메인 브릿지가 받을 토픽 이름으로 단순화
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # 브릿지에서 remap
        self.state_pub = self.create_publisher(String, 'state', 10)  # 브릿지가 받음
        self.angle_error_pub = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_pub = self.create_publisher(Float64, 'distance_error', 10)
        self.camera_pose_pub = self.create_publisher(PoseStamped, 'camera_pose', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        
        # 추가 GUI 모니터링용 발행자들
        self.current_cmd_pub = self.create_publisher(Twist, 'current_cmd_vel', 10)
        self.pid_debug_pub = self.create_publisher(String, 'pid_debug', 10)
        
        # 제어 루프 타이머
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info(f"Robot {self.robot_id} PID Controller 초기화 완료")
        
    def update_pid_parameters(self):
        """파라미터에서 PID 값 업데이트"""
        try:
            angular_kp = self.get_parameter('angular_kp').get_parameter_value().double_value
            angular_ki = self.get_parameter('angular_ki').get_parameter_value().double_value
            angular_kd = self.get_parameter('angular_kd').get_parameter_value().double_value
            
            linear_kp = self.get_parameter('linear_kp').get_parameter_value().double_value
            linear_ki = self.get_parameter('linear_ki').get_parameter_value().double_value
            linear_kd = self.get_parameter('linear_kd').get_parameter_value().double_value
            
            self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
            self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
            
            # PID 컨트롤러 업데이트
            self.angular_pid.kp = angular_kp
            self.angular_pid.ki = angular_ki
            self.angular_pid.kd = angular_kd
            
            self.linear_pid.kp = linear_kp
            self.linear_pid.ki = linear_ki
            self.linear_pid.kd = linear_kd
            
        except Exception as e:
            self.get_logger().warn(f"파라미터 업데이트 실패: {e}")
    
    def goal_callback(self, msg):
        """목표 위치 콜백"""
        self.get_logger().info(f'=== Robot {self.robot_id} 목표 수신됨! ===')
        self.get_logger().info(f'목표 위치: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.get_logger().info(f'목표 자세: z={msg.pose.orientation.z:.3f}, w={msg.pose.orientation.w:.3f}')
        
        self.goal_pose = msg.pose
        self.current_state = self.ROTATE_TO_GOAL
        
        # 상태 변경 확인 로그
        self.get_logger().info(f'상태 변경: IDLE → {self.current_state}')
        
        # PID 컨트롤러 리셋
        self.angular_pid.reset()
        self.linear_pid.reset()
        
        # target_pose 발행 (GUI용) - 현재 추적 중인 목표 알림
        self.target_pose_pub.publish(msg)
        
        # 즉시 상태 발행하여 GUI 업데이트
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)
    
    def odom_callback(self, msg):
        """오도메트리 콜백"""
        self.current_pose = msg.pose.pose
        
        # 쿼터니언에서 yaw 각도 추출
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 주기적으로 현재 위치 로그 (10번에 한 번)
        if not hasattr(self, 'odom_count'):
            self.odom_count = 0
        self.odom_count += 1
        
        if self.odom_count % 50 == 0:  # 2.5초마다 (50Hz * 50 = 2.5초)
            self.get_logger().info(
                f'Robot {self.robot_id} 현재 위치: ({self.current_pose.position.x:.3f}, {self.current_pose.position.y:.3f}), '
                f'yaw: {math.degrees(self.current_yaw):.1f}°'
            )
        
        # camera_pose 발행 (GUI용)
        camera_pose_msg = PoseStamped()
        camera_pose_msg.header = msg.header
        camera_pose_msg.pose = self.current_pose
        self.camera_pose_pub.publish(camera_pose_msg)
    
    def control_loop(self):
        """메인 제어 루프"""
        # 상태 발행 (항상)
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)
        
        if self.goal_pose is None or self.current_pose is None:
            # 디버깅: 어떤 조건이 안 맞는지 확인
            if not hasattr(self, 'debug_count'):
                self.debug_count = 0
            self.debug_count += 1
            
            if self.debug_count % 100 == 0:  # 5초마다
                self.get_logger().info(
                    f'Robot {self.robot_id} 대기 중 - goal_pose: {self.goal_pose is not None}, '
                    f'current_pose: {self.current_pose is not None}'
                )
            return
        
        # 파라미터 업데이트
        self.update_pid_parameters()
        
        # 거리와 각도 계산
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)
        
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - self.current_yaw)
        
        # 목표 자세 (쿼터니언에서 yaw 추출)
        goal_q = self.goal_pose.orientation
        goal_siny_cosp = 2.0 * (goal_q.w * goal_q.z + goal_q.x * goal_q.y)
        goal_cosy_cosp = 1.0 - 2.0 * (goal_q.y * goal_q.y + goal_q.z * goal_q.z)
        goal_yaw = math.atan2(goal_siny_cosp, goal_cosy_cosp)
        final_yaw_error = normalize_angle(goal_yaw - self.current_yaw)
        
        # 오차 발행 (GUI용)
        angle_error_msg = Float64()
        angle_error_msg.data = yaw_error
        self.angle_error_pub.publish(angle_error_msg)
        
        distance_error_msg = Float64()
        distance_error_msg.data = distance
        self.distance_error_pub.publish(distance_error_msg)
        
        # camera_pose 발행 (현재 위치를 GUI에 전달)
        camera_pose_msg = PoseStamped()
        camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
        camera_pose_msg.header.frame_id = 'map'
        camera_pose_msg.pose = self.current_pose
        self.camera_pose_pub.publish(camera_pose_msg)
        
        # 상태머신 실행 (상세한 로그 추가)
        cmd = Twist()
        
        if self.current_state == self.ROTATE_TO_GOAL:
            if abs(yaw_error) < self.angle_tolerance:
                self.current_state = self.MOVE_TO_GOAL
                self.linear_pid.reset()
                self.get_logger().info("✅ 목표 방향으로 회전 완료 → MOVE_TO_GOAL")
            else:
                cmd.angular.z = self.angular_pid.update(yaw_error)
                
        elif self.current_state == self.MOVE_TO_GOAL:
            if distance < self.distance_tolerance:
                self.current_state = self.ROTATE_TO_FINAL
                self.angular_pid.reset()
                self.get_logger().info("✅ 목표 위치 도달 → ROTATE_TO_FINAL")
            else:
                # 이동 중 방향 보정
                if abs(yaw_error) > self.angle_tolerance * 2:
                    self.current_state = self.ROTATE_TO_GOAL
                    self.angular_pid.reset()
                    self.get_logger().info("방향 재조정 필요 → ROTATE_TO_GOAL")
                else:
                    cmd.linear.x = self.linear_pid.update(distance)
                    cmd.angular.z = self.angular_pid.update(yaw_error) * 0.3
                    
        elif self.current_state == self.ROTATE_TO_FINAL:
            if abs(final_yaw_error) < self.angle_tolerance:
                self.current_state = self.GOAL_REACHED
                self.get_logger().info("✅ 최종 자세 도달 → GOAL_REACHED")
            else:
                cmd.angular.z = self.angular_pid.update(final_yaw_error)
                
        elif self.current_state == self.GOAL_REACHED:
            # 목표 도달 후 대기
            self.current_state = self.IDLE
            self.goal_pose = None
            self.get_logger().info(f"🎯 Robot {self.robot_id} 목표 완료! → IDLE 상태로 전환")
            
        # 명령 발행
        self.cmd_pub.publish(cmd)
        
        # GUI용 추가 정보 발행
        self.current_cmd_pub.publish(cmd)  # 현재 제어 명령
        
        # PID 디버그 정보 발행
        debug_msg = String()
        debug_msg.data = f"robot_{self.robot_id}:state:{self.current_state},linear_cmd:{cmd.linear.x:.3f},angular_cmd:{cmd.angular.z:.3f},distance:{distance:.3f},yaw_error:{math.degrees(yaw_error):.1f}"
        self.pid_debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotGoalController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트로 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()