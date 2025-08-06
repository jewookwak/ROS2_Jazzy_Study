# central_waypoint_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Header

class CentralWaypointPublisher(Node):
    def __init__(self):
        super().__init__('central_waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/pinky1/waypoint_cmd', 10)

    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.frame_id = 'map'

        pose1 = Pose()
        pose1.position.x = 1.0
        pose1.position.y = 1.0

        pose2 = Pose()
        pose2.position.x = 2.0
        pose2.position.y = 1.0

        pose3 = Pose()
        pose3.position.x = 2.0
        pose3.position.y = 2.0

        msg.poses = [pose1, pose2, pose3]
        self.publisher_.publish(msg)
        self.get_logger().info('Sent waypoints to pinky1')

def main():
    rclpy.init()
    node = CentralWaypointPublisher()
    node.publish_waypoints()
    rclpy.spin_once(node, timeout_sec=1.0)  # 한번만 실행하고 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
