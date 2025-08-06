import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped


class FollowWaypointsClient(Node):
    def __init__(self):
        super().__init__('follow_waypoints_client')
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.goal_pose_list = self.create_goal_poses()

        self._action_client.wait_for_server(timeout_sec=10.0)

        if not self._action_client.server_is_ready():
            self.get_logger().error('Action server not available!')
            rclpy.shutdown()
            return

        self.get_logger().info('Action server available!')
        self.send_goal()

    def create_goal_poses(self):
        poses = []

        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 2.0
        pose1.pose.position.y = 0.0
        pose1.pose.orientation.w = 0.707
        pose1.pose.orientation.z = 0.707
        poses.append(pose1)

        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = 3.5
        pose2.pose.position.y = 0.3
        pose2.pose.orientation.w = 0.707
        pose2.pose.orientation.z = 0.707
        poses.append(pose2)

        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.pose.position.x = 3.2
        pose3.pose.position.y = -1.1
        pose3.pose.orientation.w = 0.707
        pose3.pose.orientation.z = 0.707
        poses.append(pose3)

        return poses

    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.goal_pose_list

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Received feedback: current_waypoint = {current_waypoint}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted by server.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client_node = FollowWaypointsClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
