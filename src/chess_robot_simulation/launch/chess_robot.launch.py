from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to the chess robot package
    try:
        chess_robot_path = get_package_share_directory('chess_robot_simulation')
    except:
        # If not installed, use current directory
        chess_robot_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        print(f"Using local chess robot path: {chess_robot_path}")

    # RViz launch
    rviz_config_path = os.path.join(chess_robot_path, 'rviz', 'chess_robot.rviz')
    if not os.path.exists(rviz_config_path):
        print(f"[Warning] RViz config file not found at: {rviz_config_path}")
        rviz_config_path = None

    if rviz_config_path:
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
            output='screen',
        )

    # 1. Robot description publisher (needs to start first)
    robot_description_publisher = Node(
        package='chess_robot_simulation',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        output='screen'
    )

    # Note: Joint states are now published directly by the chess_robot_node

    # 3. Robot state publisher (transforms joint states to TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[('/robot_description', '/robot_description')]
    )

    # 4. Gripper control node (handles gripper open/close operations)
    gripper_control_node = Node(
        package='chess_robot_simulation',
        executable='gripper_control',
        name='gripper_control_node',
        output='screen'
    )

    # 5. Chess robot node (main controller - handles moves and robot control)
    chess_robot_node = Node(
        package='chess_robot_simulation',
        executable='chess_robot_node',
        name='chess_robot_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # Optional: Launch move_group (MoveIt planning) if available
    # This is typically launched separately, but we can try to include it
    try:
        moveit_launch = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                os.path.join(chess_robot_path, 'config', 'chess_robot_moveit.yaml'),
                {'use_sim_time': False}
            ]
        )
        moveit_available = True
        print("[Info] MoveIt move_group will be launched")
    except:
        moveit_available = False
        print("[Warning] MoveIt move_group not available - using simulation mode")

    # Create launch description with proper sequencing
    launch_actions = []
    
    # Start core robot components first
    launch_actions.append(robot_description_publisher)
    launch_actions.append(TimerAction(period=1.0, actions=[robot_state_publisher]))  # Wait 1s
    
    # Start MoveIt if available
    if moveit_available:
        launch_actions.append(TimerAction(period=2.0, actions=[moveit_launch]))  # Wait 2s
    
    # Start application nodes
    launch_actions.append(TimerAction(period=3.0, actions=[gripper_control_node]))  # Wait 3s
    launch_actions.append(TimerAction(period=4.0, actions=[chess_robot_node]))     # Wait 4s
    
    # Start visualization
    launch_actions.append(TimerAction(period=5.0, actions=[rviz_node]))            # Wait 5s
    
    # Success message
    launch_actions.append(TimerAction(
        period=6.0, 
        actions=[LogInfo(msg="=== Chess Robot System Fully Launched ===\n"
                           "Ready for chess moves! Use:\n"
                           "  ros2 run chess_robot_simulation chess_move_client\n"
                           "Or try a move: ros2 run chess_robot_simulation chess_move_client e2e4")]
    ))
    
    return LaunchDescription(launch_actions)
