import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, TransformStamped
from std_msgs.msg import Header, ColorRGBA
from tf2_ros import TransformBroadcaster
import time
import chess
import numpy as np
from moveit_msgs.msg import DisplayRobotState
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
import quaternion  # for handling rotations

# Define the board dimensions
SQUARE_SIZE = 0.05  # meters
BOARD_SIZE = 8 * SQUARE_SIZE
BOARD_Z = 0.01  # Board thickness

# Define robot base position relative to board
ROBOT_BASE_X = -0.3  # 30cm to the left of the board
ROBOT_BASE_Y = BOARD_SIZE / 2  # Centered with the board
ROBOT_BASE_Z = 0.0  # At the same height as the board base

# Define colors
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
BLACK = ColorRGBA(r=0.2, g=0.2, b=0.2, a=1.0)
LIGHT_SQUARE = ColorRGBA(r=0.9, g=0.8, b=0.6, a=1.0)
DARK_SQUARE = ColorRGBA(r=0.5, g=0.4, b=0.3, a=1.0)

class ChessRobotNode(Node):
    def __init__(self):
        super().__init__('chess_robot_node')
        
        # Create publishers for visualization
        self.board_publisher = self.create_publisher(Marker, 'chess_board_marker', 10)
        self.pieces_publisher = self.create_publisher(MarkerArray, 'chess_pieces_markers', 10)
        
        # Initialize MoveIt
        self.robot = MoveItPy(node_name="chess_robot")
        self.planning_component = self.robot.get_planning_component("ur5_arm")
        
        # Initialize chess board
        self.board = chess.Board()
        self.board_marker = None
        self.piece_markers = MarkerArray()
        
        # Setup TF broadcaster for robot base
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timers
        self.create_timer(0.1, self.publish_board)
        self.create_timer(0.1, self.publish_pieces)
        self.create_timer(0.1, self.publish_robot_base_tf)
        
        # Initial board setup
        self.create_board_marker()
        self.update_piece_markers()
        
        self.get_logger().info('Chess Robot Node Started')

    def publish_robot_base_tf(self):
        # Publish the transform from world to robot base
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot_base'
        
        t.transform.translation.x = ROBOT_BASE_X
        t.transform.translation.y = ROBOT_BASE_Y
        t.transform.translation.z = ROBOT_BASE_Z
        
        # Robot faces the chess board (rotated 90 degrees around Z)
        q = quaternion.from_euler_angles(0, 0, np.pi/2)
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        
        self.tf_broadcaster.sendTransform(t)

    def create_board_marker(self):
        # Create a marker for the chess board
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "world"
        marker.ns = "chess_board"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set board position
        marker.pose.position.x = BOARD_SIZE / 2
        marker.pose.position.y = BOARD_SIZE / 2
        marker.pose.position.z = BOARD_Z / 2
        marker.pose.orientation.w = 1.0
        
        # Set board size
        marker.scale.x = BOARD_SIZE
        marker.scale.y = BOARD_SIZE
        marker.scale.z = BOARD_Z
        
        # Set board color (light brown)
        marker.color.r = 0.8
        marker.color.g = 0.6
        marker.color.b = 0.4
        marker.color.a = 1.0
        
        self.board_marker = marker
        
        # Create square markers
        for i in range(8):
            for j in range(8):
                square_marker = Marker()
                square_marker.header = Header()
                square_marker.header.frame_id = "world"
                square_marker.ns = "chess_squares"
                square_marker.id = i * 8 + j + 1
                square_marker.type = Marker.CUBE
                square_marker.action = Marker.ADD
                
                # Position in the center of each square
                square_marker.pose.position.x = j * SQUARE_SIZE + SQUARE_SIZE / 2
                square_marker.pose.position.y = i * SQUARE_SIZE + SQUARE_SIZE / 2
                square_marker.pose.position.z = BOARD_Z + 0.001  # Slightly above the board
                square_marker.pose.orientation.w = 1.0
                
                # Slightly smaller than full square size
                square_marker.scale.x = SQUARE_SIZE * 0.95
                square_marker.scale.y = SQUARE_SIZE * 0.95
                square_marker.scale.z = 0.002  # Very thin
                
                # Alternating colors
                if (i + j) % 2 == 0:
                    square_marker.color = LIGHT_SQUARE
                else:
                    square_marker.color = DARK_SQUARE
                    
                self.piece_markers.markers.append(square_marker)

    def get_piece_pose(self, piece_idx, piece_type, is_white):
        # Calculate position based on piece index
        rank = piece_idx // 8
        file = piece_idx % 8
        
        # Calculate position in 3D space
        pose = Pose()
        pose.position.x = file * SQUARE_SIZE + SQUARE_SIZE / 2
        pose.position.y = rank * SQUARE_SIZE + SQUARE_SIZE / 2
        pose.position.z = BOARD_Z  # Start at board level
        
        # Set orientation (default facing up)
        pose.orientation.w = 1.0
        
        # Adjust height based on piece type
        if piece_type == 'p':  # Pawn
            pose.position.z += 0.03
        elif piece_type in ['r', 'n', 'b']:  # Rook, Knight, Bishop
            pose.position.z += 0.035
        elif piece_type == 'q':  # Queen
            pose.position.z += 0.045
        elif piece_type == 'k':  # King
            pose.position.z += 0.05
            
        return pose

    async def move_piece(self, from_square, to_square):
        """Move a chess piece using the robot arm"""
        # Convert chess notation to coordinates
        from_file = chess.square_file(from_square)
        from_rank = chess.square_rank(from_square)
        to_file = chess.square_file(to_square)
        to_rank = chess.square_rank(to_square)
        
        # Get 3D positions
        from_pos = self.get_square_position(from_file, from_rank)
        to_pos = self.get_square_position(to_file, to_rank)
        
        # Plan and execute pick motion
        await self.move_arm_to_position(from_pos)
        # Here would be gripper control code
        
        # Plan and execute place motion
        await self.move_arm_to_position(to_pos)
        # Here would be gripper release code
        
        # Return to home position
        await self.move_to_home_position()

    def get_square_position(self, file, rank):
        """Convert chess square coordinates to 3D position"""
        return {
            'x': file * SQUARE_SIZE + SQUARE_SIZE / 2,
            'y': rank * SQUARE_SIZE + SQUARE_SIZE / 2,
            'z': BOARD_Z + 0.1  # Hover above the board
        }

    async def move_arm_to_position(self, position):
        """Plan and execute robot arm movement to a position"""
        target_pose = Pose()
        target_pose.position.x = position['x']
        target_pose.position.y = position['y']
        target_pose.position.z = position['z']
        
        # Orient the end-effector downward
        q = quaternion.from_euler_angles(np.pi, 0, 0)  # Pointing down
        target_pose.orientation.x = q.x
        target_pose.orientation.y = q.y
        target_pose.orientation.z = q.z
        target_pose.orientation.w = q.w
        
        # Plan to target
        self.planning_component.set_goal_pose(target_pose, "ee_link")
        plan_result = await self.planning_component.plan_async()
        
        if plan_result.success:
            # Execute the plan
            await self.planning_component.execute_async()
        else:
            self.get_logger().error("Failed to plan path to target")

    async def move_to_home_position(self):
        """Move the robot arm to its home position"""
        self.planning_component.set_named_target("home")
        plan_result = await self.planning_component.plan_async()
        
        if plan_result.success:
            await self.planning_component.execute_async()
        else:
            self.get_logger().error("Failed to plan path to home position")

    def update_piece_markers(self):
        # (Same as original implementation)
        pass

    def create_piece_marker(self, marker_id, piece_type, is_white, square_idx):
        # (Same as original implementation)
        pass

    def publish_board(self):
        if self.board_marker is not None:
            self.board_publisher.publish(self.board_marker)

    def publish_pieces(self):
        self.update_piece_markers()
        self.pieces_publisher.publish(self.piece_markers)

def main(args=None):
    rclpy.init(args=args)
    node = ChessRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()