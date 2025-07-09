#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64, String, Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time

# Assuming you have a custom service definition
# from chess_robot_msgs.srv import ChessMove

class ChessRobotNode(Node):
    def __init__(self):
        super().__init__('chess_robot_node')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.gripper_pub = self.create_publisher(Float64, '/gripper_position', 10)
        self.attach_pub = self.create_publisher(String, '/attach_piece', 10)
        self.detach_pub = self.create_publisher(Bool, '/detach_piece', 10)
        
        # TF broadcaster for piece tracking
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Chess board setup - board positioned to the side of robot
        self.square_size = 0.05
        self.board_offset_x = 0.3  # Board 30cm to the right of robot
        self.board_offset_y = 0.0  # Centered in Y
        self.board_height = 0.02
        self.piece_height = 0.03
        self.pickup_height = 0.15
        self.approach_height = 0.08
        
        # Robot state
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_position = 0.0
        self.attached_piece = None
        self.piece_positions = self._initialize_pieces()
        
        # Subscriber for chess move commands
        self.move_sub = self.create_subscription(
            String,
            '/chess_move_command',
            self.handle_chess_move,
            10
        )
        
        # Timer for continuous updates
        self.timer = self.create_timer(0.02, self.update_loop)
        
        self.get_logger().info('Chess Robot Node initialized')
        self.get_logger().info(f'Board positioned at x={self.board_offset_x}, y={self.board_offset_y}')
        self.get_logger().info('Waiting for chess moves on /chess_move_command topic...')
    
    def _initialize_pieces(self):
        """Initialize chess piece positions"""
        pieces = {}
        # Add all chess pieces at their starting positions
        # White pieces
        for i in range(8):
            pieces[f'white_pawn_{i}'] = self._square_to_position(i, 1)
        pieces['white_rook_0'] = self._square_to_position(0, 0)
        pieces['white_rook_1'] = self._square_to_position(7, 0)
        pieces['white_knight_0'] = self._square_to_position(1, 0)
        pieces['white_knight_1'] = self._square_to_position(6, 0)
        pieces['white_bishop_0'] = self._square_to_position(2, 0)
        pieces['white_bishop_1'] = self._square_to_position(5, 0)
        pieces['white_queen'] = self._square_to_position(3, 0)
        pieces['white_king'] = self._square_to_position(4, 0)
        
        # Black pieces
        for i in range(8):
            pieces[f'black_pawn_{i}'] = self._square_to_position(i, 6)
        pieces['black_rook_0'] = self._square_to_position(0, 7)
        pieces['black_rook_1'] = self._square_to_position(7, 7)
        pieces['black_knight_0'] = self._square_to_position(1, 7)
        pieces['black_knight_1'] = self._square_to_position(6, 7)
        pieces['black_bishop_0'] = self._square_to_position(2, 7)
        pieces['black_bishop_1'] = self._square_to_position(5, 7)
        pieces['black_queen'] = self._square_to_position(3, 7)
        pieces['black_king'] = self._square_to_position(4, 7)
        
        return pieces
    
    def _square_to_position(self, col, row):
        """Convert chess square to 3D position"""
        x = self.board_offset_x + (col - 3.5) * self.square_size
        y = self.board_offset_y + (row - 3.5) * self.square_size
        z = self.board_height + self.piece_height / 2
        return [x, y, z]
    
    def _parse_chess_notation(self, move):
        """Parse chess notation (e.g., 'e2e4') to board coordinates"""
        if len(move) != 4:
            return None
        
        from_col = ord(move[0].lower()) - ord('a')
        from_row = int(move[1]) - 1
        to_col = ord(move[2].lower()) - ord('a')
        to_row = int(move[3]) - 1
        
        if not (0 <= from_col < 8 and 0 <= from_row < 8 and
                0 <= to_col < 8 and 0 <= to_row < 8):
            return None
        
        return (from_col, from_row, to_col, to_row)
    
    def _find_piece_at_square(self, col, row):
        """Find which piece is at the given square"""
        target_pos = self._square_to_position(col, row)
        
        for piece_name, pos in self.piece_positions.items():
            if (abs(pos[0] - target_pos[0]) < 0.01 and 
                abs(pos[1] - target_pos[1]) < 0.01):
                return piece_name
        
        return None
    
    def handle_chess_move(self, msg):
        """Handle incoming chess move commands"""
        move_str = msg.data
        self.get_logger().info(f"Received move command: {move_str}")
        self.execute_chess_move(move_str)
    
    def execute_chess_move(self, move_str):
        """Execute a chess move given notation like 'e2e4'"""
        coords = self._parse_chess_notation(move_str)
        if not coords:
            self.get_logger().error(f"Invalid move notation: {move_str}")
            return False
        
        from_col, from_row, to_col, to_row = coords
        
        # Find piece at source square
        piece_name = self._find_piece_at_square(from_col, from_row)
        if not piece_name:
            self.get_logger().error(f"No piece at {move_str[:2]}")
            return False
        
        # Execute the move
        self.get_logger().info(f"Moving {piece_name} from {move_str[:2]} to {move_str[2:]}")
        
        # Get positions
        from_pos = self._square_to_position(from_col, from_row)
        to_pos = self._square_to_position(to_col, to_row)
        
        # Execute pick and place
        return self.pick_and_place(piece_name, from_pos, to_pos)
    
    def pick_and_place(self, piece_name, from_pos, to_pos):
        """Execute pick and place operation"""
        try:
            # Move to approach position above piece
            approach_pos = [from_pos[0], from_pos[1], self.approach_height]
            self.move_to_position(approach_pos, [0, np.pi, 0])
            
            # Open gripper
            self.set_gripper(0.04)
            time.sleep(0.5)
            
            # Move down to piece
            pick_pos = [from_pos[0], from_pos[1], from_pos[2] + 0.01]
            self.move_to_position(pick_pos, [0, np.pi, 0])
            
            # Close gripper and attach piece
            self.set_gripper(0.01)
            self.attach_piece(piece_name)
            time.sleep(0.5)
            
            # Lift piece
            lift_pos = [from_pos[0], from_pos[1], self.pickup_height]
            self.move_to_position(lift_pos, [0, np.pi, 0])
            
            # Move to above destination
            transit_pos = [to_pos[0], to_pos[1], self.pickup_height]
            self.move_to_position(transit_pos, [0, np.pi, 0])
            
            # Lower to place position
            place_pos = [to_pos[0], to_pos[1], to_pos[2] + 0.01]
            self.move_to_position(place_pos, [0, np.pi, 0])
            
            # Open gripper and detach piece
            self.set_gripper(0.04)
            self.detach_piece()
            self.piece_positions[piece_name] = to_pos
            time.sleep(0.5)
            
            # Lift back up
            self.move_to_position(transit_pos, [0, np.pi, 0])
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Pick and place failed: {str(e)}")
            return False
    
    def attach_piece(self, piece_name):
        """Attach a piece to the gripper"""
        self.attached_piece = piece_name
        msg = String()
        msg.data = piece_name
        self.attach_pub.publish(msg)
        self.get_logger().info(f"Attached piece: {piece_name}")
    
    def detach_piece(self):
        """Detach the current piece from gripper"""
        if self.attached_piece:
            self.get_logger().info(f"Detaching piece: {self.attached_piece}")
            self.attached_piece = None
            msg = Bool()
            msg.data = True
            self.detach_pub.publish(msg)
    
    def move_to_position(self, position, orientation):
        """Move arm to target position with IK"""
        # Calculate inverse kinematics
        joints = self.calculate_ik(position, orientation)
        if joints is None:
            self.get_logger().error(f"IK failed for position {position}")
            return False
        
        # Interpolate movement
        steps = 50
        start_joints = self.current_joints.copy()
        
        for i in range(steps):
            t = (i + 1) / steps
            # Smooth interpolation
            t_smooth = 0.5 * (1 - np.cos(np.pi * t))
            
            for j in range(6):
                self.current_joints[j] = start_joints[j] + (joints[j] - start_joints[j]) * t_smooth
            
            # Update attached piece position if holding one
            if self.attached_piece:
                self.update_attached_piece_position()
            
            # Publish current joint state
            self.publish_joint_states()
            time.sleep(0.02)
        
        return True
    
    def update_attached_piece_position(self):
        """Update position of attached piece based on gripper position"""
        if not self.attached_piece:
            return
        
        # Calculate end-effector position from forward kinematics
        ee_pos = self.calculate_fk(self.current_joints)
        
        # Update piece position (piece is below gripper)
        piece_offset = [0, 0, -0.03]  # Offset from gripper center
        self.piece_positions[self.attached_piece] = [
            ee_pos[0] + piece_offset[0],
            ee_pos[1] + piece_offset[1],
            ee_pos[2] + piece_offset[2]
        ]
    
    def calculate_ik(self, position, orientation):
        """Simple IK calculation (placeholder - implement actual IK)"""
        # This is a simplified IK - you should implement proper IK for your robot
        joints = [0.0] * 6
        
        # Calculate base rotation to point at target
        joints[0] = np.arctan2(position[1], position[0])
        
        # Simple approximation for other joints based on position
        dist_xy = np.sqrt(position[0]**2 + position[1]**2)
        height = position[2]
        
        # These values would need to be calculated properly based on your robot's kinematics
        # This is just a placeholder that gives reasonable looking motion
        if height > 0.1:
            joints[1] = -np.pi/6
            joints[2] = np.pi/3
            joints[3] = -np.pi/6
        else:
            joints[1] = -np.pi/3
            joints[2] = np.pi/2
            joints[3] = -np.pi/4
        
        joints[4] = 0.0
        joints[5] = 0.0
        
        return joints
    
    def calculate_fk(self, joints):
        """Simple FK calculation (placeholder - implement actual FK)"""
        # This is a simplified FK - you should implement proper FK for your robot
        # Link lengths (adjust based on your robot)
        l1 = 0.1   # Base to shoulder height
        l2 = 0.15  # Upper arm length
        l3 = 0.15  # Forearm length
        l4 = 0.1   # Wrist to end-effector
        
        # Simple 2D projection for x,y based on base rotation
        reach = l2 * np.cos(joints[1]) + l3 * np.cos(joints[1] + joints[2]) + l4
        x = reach * np.cos(joints[0])
        y = reach * np.sin(joints[0])
        
        # Height calculation
        z = l1 + l2 * np.sin(joints[1]) + l3 * np.sin(joints[1] + joints[2])
        
        return [x, y, z]
    
    def set_gripper(self, position):
        """Set gripper position"""
        self.gripper_position = position
        msg = Float64()
        msg.data = position
        self.gripper_pub.publish(msg)
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state.position = self.current_joints
        self.joint_pub.publish(joint_state)
    
    def update_loop(self):
        """Main update loop - publish joint states and piece transforms"""
        # Publish joint states
        self.publish_joint_states()
        
        # Broadcast piece transforms
        for piece_name, position in self.piece_positions.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = piece_name
            t.transform.translation.x = position[0]
            t.transform.translation.y = position[1]
            t.transform.translation.z = position[2]
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

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