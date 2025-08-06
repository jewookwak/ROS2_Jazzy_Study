import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose3', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        if self.count > 0:
            return  # 한 번만 보냄

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.1114
        goal_pose.pose.position.y = -0.2883
        goal_pose.pose.orientation.z = -0.0046
        goal_pose.pose.orientation.w = 0.9999  # 회전 없음

        self.publisher_.publish(goal_pose)
        self.get_logger().info('Published goal_pose')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
