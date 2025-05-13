from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the UR5 description package
    ur_description_path = get_package_share_directory('ur_description')

    # Get path to the chess robot package
    chess_robot_path = get_package_share_directory('chess_robot_simulation')

    # Load UR5 description launch file (make sure this file exists)
    ur5_launch_path = os.path.join(ur_description_path, 'launch', 'ur5_description.launch.py')
    if not os.path.exists(ur5_launch_path):
        raise FileNotFoundError(f"Could not find UR5 launch file at: {ur5_launch_path}")

    ur5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur5_launch_path)
    )

    # RViz launch (optional, comment out if not needed)
    rviz_config_path = os.path.join(chess_robot_path, 'rviz', 'chess_robot.rviz')
    if not os.path.exists(rviz_config_path):
        print(f"[Warning] RViz config file not found at: {rviz_config_path}")
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    else:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )

    # Launch the chess robot node (make sure it's installed with entry_points!)
    chess_robot_node = Node(
        package='chess_robot_simulation',
        executable='chess_robot_node',
        name='chess_robot_node',
        output='screen'
    )

    return LaunchDescription([
        ur5_launch,
        rviz_node,
        chess_robot_node,
    ])
