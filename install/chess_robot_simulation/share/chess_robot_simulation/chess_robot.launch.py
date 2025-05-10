from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chess_robot_simulation',
            executable='chess_robot_node',
            name='chess_robot',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/opt/ros/humble/share/rviz2/default.rviz'],
            output='screen',
        ),
    ])