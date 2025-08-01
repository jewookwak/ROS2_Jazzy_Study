from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapf_planner',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen'
        ),
        Node(
            package='mapf_planner',
            executable='tracked_path_publisher',
            name='tracked_path_publisher',
            output='screen'
        )
    ])
