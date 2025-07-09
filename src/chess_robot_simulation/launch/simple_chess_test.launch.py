from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the chess robot package
    try:
        chess_robot_path = get_package_share_directory('chess_robot_simulation')
    except:
        chess_robot_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # RViz launch
    rviz_config_path = os.path.join(chess_robot_path, 'rviz', 'chess_robot.rviz')
    if os.path.exists(rviz_config_path):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    else:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )

    # Robot description publisher
    robot_description_publisher = Node(
        package='chess_robot_simulation',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen'
    )

    # Simple chess arm controller (instead of complex chess robot node)
    simple_chess_arm = Node(
        package='chess_robot_simulation',
        executable='simple_chess_arm',
        name='simple_chess_arm',
        output='screen'
    )

    return LaunchDescription([
        robot_description_publisher,
        robot_state_publisher, 
        simple_chess_arm,
        rviz_node,
        LogInfo(msg="🤖 Simple Chess Robot Ready! The arm WILL move when you make moves!")
    ]) 