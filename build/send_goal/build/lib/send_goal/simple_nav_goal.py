import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class SimpleNavGoal(Node):
    def __init__(self):
        super().__init__('simple_nav_goal')
        # ActionClient 생성: Nav2의 navigate_to_pose 액션 서버와 통신
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 1초 후 send_goal 호출
        self._timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        # 액션 서버가 준비될 때까지 대기
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
        # 목표 메시지 생성
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # 맵 프레임 설정
        pose.header.stamp = self.get_clock().now().to_msg()
        # 목표 위치와 자세 설정
        pose.pose.position.x = 2.0  # 예시: x
        pose.pose.position.y = 0.0  # 예시: y
        pose.pose.orientation.w = 1.0  # 방향 (Quaternion)
        goal_msg.pose = pose
        # 액션 서버로 목표 전송 (비동기)
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        # 타이머는 첫 전송 후 취소
        self._timer.cancel()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        # 결과 수신 대기
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # 피드백 처리
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current pose: {feedback.current_pose}')

    def get_result_callback(self, future):
        # 결과 처리 후 노드 종료
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavGoal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
