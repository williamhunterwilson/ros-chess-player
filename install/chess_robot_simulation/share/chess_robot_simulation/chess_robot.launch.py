from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the UR5 description package
    ur_description_path = get_package_share_directory('ur_description')
    
    # Get paths to the chess robot package
    chess_robot_path = get_package_share_directory('chess_robot_simulation')
    
    # Load UR5 description launch
    ur5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_description_path, 'launch', 'ur5_description.launch.py')
        )
    )
    
    # Launch RViz with custom configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(chess_robot_path, 'config', 'chess_robot.rviz')],
        output='screen'
    )
    
    # Launch the chess robot node
    chess_robot_node = Node(
        package='chess_robot_simulation',
        executable='chess_robot_node',
        name='chess_robot_node',
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        ur5_launch,
        rviz_node,
        chess_robot_node,
    ])
