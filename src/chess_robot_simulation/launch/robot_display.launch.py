import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the package directory
    pkg_dir = get_package_share_directory('chess_robot_simulation')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'chess_robot.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    
    # Robot description parameter
    robot_description = {'robot_description': robot_description_content}
    
    # RViz config file
    rviz_config = os.path.join(pkg_dir, 'config', 'chess_robot.rviz')
    
    # If config doesn't exist, use default
    if not os.path.exists(rviz_config):
        rviz_config = None
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        ),
        
        # Joint state publisher
        Node(
            package='chess_robot_simulation',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Main robot controller
        Node(
            package='chess_robot_simulation',
            executable='chess_robot_node',
            name='chess_robot_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Piece attachment manager
        Node(
            package='chess_robot_simulation',
            executable='piece_attachment_manager',
            name='piece_attachment_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Gripper controller
        Node(
            package='chess_robot_simulation',
            executable='gripper_control',
            name='gripper_control',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Chess visualization
        Node(
            package='chess_robot_simulation',
            executable='chess_visualization',
            name='chess_visualization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # RViz with custom config if available
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if rviz_config else [],
            parameters=[{'use_sim_time': use_sim_time}],
            on_exit=ExecuteProcess(
                cmd=['echo', 'RViz closed'],
                output='screen'
            )
        ),
    ])