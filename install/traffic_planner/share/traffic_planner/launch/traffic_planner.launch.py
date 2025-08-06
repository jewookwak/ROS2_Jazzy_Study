from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments (파라미터로 전달 가능)
        DeclareLaunchArgument('num_robots', default_value='2', 
                            description='Number of robots'),
        DeclareLaunchArgument('timeout_seconds', default_value='10.0',
                            description='Timeout for each waypoint'),
        DeclareLaunchArgument('resolution', default_value='0.1',
                            description='Map resolution'),
        
        # TrafficPlanner 노드
        Node(
            package='traffic_planner',
            executable='traffic_path_planner',
            name='traffic_path_planner_node',
            parameters=[{
                'num_robots': LaunchConfiguration('num_robots'),
                'timeout_seconds': LaunchConfiguration('timeout_seconds'),
                'resolution': LaunchConfiguration('resolution'),
                # 기본 시작점/목표점 (double 타입으로 명시)
                'start_positions': [2.0, 19.0, 4.0, 19.0, 6.0, 19.0],
                'goal_positions': [3.0, 11.0, 5.0, 11.0, 7.0, 11.0]
            }],
            output='screen'
        )
    ])