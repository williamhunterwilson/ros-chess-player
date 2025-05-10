import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header, ColorRGBA
import time
import chess
import numpy as np

# Define the board dimensions
SQUARE_SIZE = 0.05  # meters
BOARD_SIZE = 8 * SQUARE_SIZE
BOARD_Z = 0.01  # Board thickness

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
        
        # Initialize chess board
        self.board = chess.Board()
        self.board_marker = None
        self.piece_markers = MarkerArray()
        
        # Create timers
        self.create_timer(0.1, self.publish_board)
        self.create_timer(0.1, self.publish_pieces)
        
        # Initial board setup
        self.create_board_marker()
        self.update_piece_markers()
        
        self.get_logger().info('Chess Robot Node Started')

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

    def update_piece_markers(self):
        # Clear existing piece markers
        for i in range(len(self.piece_markers.markers)):
            if self.piece_markers.markers[i].ns == "chess_pieces":
                self.piece_markers.markers[i].action = Marker.DELETE
        
        # Add markers for each piece on the board
        piece_id = 100  # Start IDs from 100 to avoid conflicts with board squares
        
        for square_idx in range(64):
            # Convert to chess notation (a1, a2, etc.)
            rank = 7 - (square_idx // 8)
            file = square_idx % 8
            square = chess.square(file, rank)
            
            piece = self.board.piece_at(square)
            if piece is not None:
                piece_type = piece.symbol().lower()
                is_white = piece.symbol().isupper()
                
                marker = self.create_piece_marker(piece_id, piece_type, is_white, square_idx)
                self.piece_markers.markers.append(marker)
                piece_id += 1

    def create_piece_marker(self, marker_id, piece_type, is_white, square_idx):
        # Create a marker for a chess piece
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "world"
        marker.ns = "chess_pieces"
        marker.id = marker_id
        marker.action = Marker.ADD
        
        # Set marker type based on piece
        if piece_type == 'p':  # Pawn
            marker.type = Marker.CYLINDER
            marker.scale.x = SQUARE_SIZE * 0.3
            marker.scale.y = SQUARE_SIZE * 0.3
            marker.scale.z = SQUARE_SIZE * 0.6
        elif piece_type == 'r':  # Rook
            marker.type = Marker.CUBE
            marker.scale.x = SQUARE_SIZE * 0.4
            marker.scale.y = SQUARE_SIZE * 0.4
            marker.scale.z = SQUARE_SIZE * 0.7
        elif piece_type == 'n':  # Knight
            # Instead of CONE (which doesn't exist in ROS2), use CYLINDER with modifications
            marker.type = Marker.CYLINDER
            marker.scale.x = SQUARE_SIZE * 0.35
            marker.scale.y = SQUARE_SIZE * 0.35
            marker.scale.z = SQUARE_SIZE * 0.7
        elif piece_type == 'b':  # Bishop
            marker.type = Marker.CYLINDER
            marker.scale.x = SQUARE_SIZE * 0.35
            marker.scale.y = SQUARE_SIZE * 0.35
            marker.scale.z = SQUARE_SIZE * 0.8
        elif piece_type == 'q':  # Queen
            marker.type = Marker.SPHERE
            marker.scale.x = SQUARE_SIZE * 0.45
            marker.scale.y = SQUARE_SIZE * 0.45
            marker.scale.z = SQUARE_SIZE * 0.9
        elif piece_type == 'k':  # King
            marker.type = Marker.CYLINDER
            marker.scale.x = SQUARE_SIZE * 0.4
            marker.scale.y = SQUARE_SIZE * 0.4
            marker.scale.z = SQUARE_SIZE * 1.0
            
        # Set position based on square index
        marker.pose = self.get_piece_pose(square_idx, piece_type, is_white)
        
        # Set color based on piece color
        if is_white:
            marker.color = WHITE
        else:
            marker.color = BLACK
            
        # Set lifetime to zero (persistent)
        marker.lifetime.sec = 0
        
        return marker

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