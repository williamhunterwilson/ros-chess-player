from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chess_robot_simulation',
            executable='minimal_node',
            name='minimal_chess_node',
            output='screen',
        ),
        Node(
            package='chess_robot_simulation',
            executable='minimal_publisher',
            name='minimal_publisher',
            output='screen',
        ),
        # Add RViz but with a configured file that doesn't require specific frames
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        # Add a command to show topics for debugging
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 2; ros2 topic list; ros2 node list'],
            output='screen',
        ),
    ])