#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf2_ros
from tf2_ros import TransformListener, Buffer

class ChessVisualization(Node):
    def __init__(self):
        super().__init__('chess_visualization')
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, 'chess_visualization', 10)
        
        # TF2 for piece positions
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Chess setup - board positioned to the side of robot
        self.square_size = 0.05
        self.board_offset_x = 0.3  # Move board 30cm to the right of robot
        self.board_offset_y = 0.0  # Centered in Y
        self.board_height = 0.02
        self.board_thickness = 0.01
        
        # Timer to update visualization
        self.timer = self.create_timer(0.1, self.publish_chess_visualization)
        
        # Piece types and their mesh approximations
        self.piece_info = {
            'pawn': {'height': 0.03, 'radius': 0.015},
            'rook': {'height': 0.04, 'radius': 0.018},
            'knight': {'height': 0.045, 'radius': 0.018},
            'bishop': {'height': 0.05, 'radius': 0.016},
            'queen': {'height': 0.055, 'radius': 0.02},
            'king': {'height': 0.06, 'radius': 0.02}
        }
        
        self.get_logger().info('Chess Visualization Node started')
        self.get_logger().info(f'Board offset: x={self.board_offset_x}, y={self.board_offset_y}')
    
    def create_chessboard_markers(self):
        """Create markers for the chessboard"""
        markers = []
        marker_id = 0
        
        # Board base
        board_marker = Marker()
        board_marker.header.frame_id = "world"
        board_marker.header.stamp = self.get_clock().now().to_msg()
        board_marker.ns = "chessboard"
        board_marker.id = marker_id
        board_marker.type = Marker.CUBE
        board_marker.action = Marker.ADD
        
        # Position board offset from robot
        board_marker.pose.position.x = self.board_offset_x
        board_marker.pose.position.y = self.board_offset_y
        board_marker.pose.position.z = self.board_height / 2.0
        board_marker.pose.orientation.w = 1.0
        
        # Size of entire board
        board_size = 8 * self.square_size
        board_marker.scale.x = board_size + 0.02  # Add border
        board_marker.scale.y = board_size + 0.02
        board_marker.scale.z = self.board_thickness
        
        # Wood color for board
        board_marker.color.r = 0.55
        board_marker.color.g = 0.27
        board_marker.color.b = 0.07
        board_marker.color.a = 1.0
        
        markers.append(board_marker)
        marker_id += 1
        
        # Create checkerboard pattern
        for row in range(8):
            for col in range(8):
                square_marker = Marker()
                square_marker.header.frame_id = "world"
                square_marker.header.stamp = self.get_clock().now().to_msg()
                square_marker.ns = "squares"
                square_marker.id = marker_id
                square_marker.type = Marker.CUBE
                square_marker.action = Marker.ADD
                
                # Position with offset
                x = self.board_offset_x + (col - 3.5) * self.square_size
                y = self.board_offset_y + (row - 3.5) * self.square_size
                square_marker.pose.position.x = x
                square_marker.pose.position.y = y
                square_marker.pose.position.z = self.board_height + 0.001  # Slightly above board
                square_marker.pose.orientation.w = 1.0
                
                # Size
                square_marker.scale.x = self.square_size * 0.95  # Slight gap between squares
                square_marker.scale.y = self.square_size * 0.95
                square_marker.scale.z = 0.001
                
                # Color - alternating black and white
                if (row + col) % 2 == 0:
                    square_marker.color.r = 0.9
                    square_marker.color.g = 0.9
                    square_marker.color.b = 0.9
                else:
                    square_marker.color.r = 0.2
                    square_marker.color.g = 0.2
                    square_marker.color.b = 0.2
                square_marker.color.a = 1.0
                
                markers.append(square_marker)
                marker_id += 1
        
        # Add coordinate labels
        for i in range(8):
            # File labels (a-h)
            label_marker = Marker()
            label_marker.header.frame_id = "world"
            label_marker.header.stamp = self.get_clock().now().to_msg()
            label_marker.ns = "labels"
            label_marker.id = marker_id
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.text = chr(ord('a') + i)
            label_marker.pose.position.x = self.board_offset_x + (i - 3.5) * self.square_size
            label_marker.pose.position.y = self.board_offset_y - 4.5 * self.square_size
            label_marker.pose.position.z = self.board_height + 0.01
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = 0.02
            label_marker.color.r = 0.0
            label_marker.color.g = 0.0
            label_marker.color.b = 0.0
            label_marker.color.a = 1.0
            markers.append(label_marker)
            marker_id += 1
            
            # Rank labels (1-8)
            label_marker = Marker()
            label_marker.header.frame_id = "world"
            label_marker.header.stamp = self.get_clock().now().to_msg()
            label_marker.ns = "labels"
            label_marker.id = marker_id
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.text = str(i + 1)
            label_marker.pose.position.x = self.board_offset_x - 4.5 * self.square_size
            label_marker.pose.position.y = self.board_offset_y + (i - 3.5) * self.square_size
            label_marker.pose.position.z = self.board_height + 0.01
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = 0.02
            label_marker.color.r = 0.0
            label_marker.color.g = 0.0
            label_marker.color.b = 0.0
            label_marker.color.a = 1.0
            markers.append(label_marker)
            marker_id += 1
        
        return markers, marker_id
    
    def create_piece_marker(self, piece_name, position, marker_id):
        """Create a marker for a chess piece"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pieces"
        marker.id = marker_id
        marker.action = Marker.ADD
        
        # Determine piece type and color
        parts = piece_name.split('_')
        color = parts[0]  # 'white' or 'black'
        piece_type = parts[1]  # 'pawn', 'rook', etc.
        
        # Get piece dimensions
        if piece_type in self.piece_info:
            info = self.piece_info[piece_type]
        else:
            info = self.piece_info['pawn']  # Default
        
        # Use cylinder for all pieces (no mesh files needed)
        marker.type = Marker.CYLINDER
        marker.scale.x = info['radius'] * 2
        marker.scale.y = info['radius'] * 2
        marker.scale.z = info['height']
        
        # Position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        # Color based on piece color
        if color == 'white':
            marker.color.r = 0.95
            marker.color.g = 0.95
            marker.color.b = 0.95
            marker.color.a = 1.0
        else:
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0
        
        # Add piece type text
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "piece_labels"
        text_marker.id = marker_id + 1000  # Offset to avoid ID conflicts
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Unicode chess symbols
        piece_symbols = {
            'pawn': '♟' if color == 'black' else '♙',
            'rook': '♜' if color == 'black' else '♖',
            'knight': '♞' if color == 'black' else '♘',
            'bishop': '♝' if color == 'black' else '♗',
            'queen': '♛' if color == 'black' else '♕',
            'king': '♚' if color == 'black' else '♔'
        }
        
        text_marker.text = piece_symbols.get(piece_type, '?')
        text_marker.pose.position.x = position[0]
        text_marker.pose.position.y = position[1]
        text_marker.pose.position.z = position[2] + info['height'] + 0.01
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.03
        text_marker.color.r = 0.5 if color == 'white' else 0.7
        text_marker.color.g = 0.5 if color == 'white' else 0.7
        text_marker.color.b = 0.5 if color == 'white' else 0.7
        text_marker.color.a = 1.0
        
        return [marker, text_marker]
    
    def get_piece_positions_from_tf(self):
        """Get chess piece positions from TF"""
        pieces = {}
        
        # List of all possible pieces
        piece_names = []
        for color in ['white', 'black']:
            for i in range(8):
                piece_names.append(f'{color}_pawn_{i}')
            for i in range(2):
                piece_names.append(f'{color}_rook_{i}')
                piece_names.append(f'{color}_knight_{i}')
                piece_names.append(f'{color}_bishop_{i}')
            piece_names.append(f'{color}_queen')
            piece_names.append(f'{color}_king')
        
        # Try to get transforms for each piece
        for piece_name in piece_names:
            try:
                t = self.tf_buffer.lookup_transform(
                    'world',
                    piece_name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.01)
                )
                pieces[piece_name] = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z
                ]
            except:
                # Piece transform not available yet
                pass
        
        return pieces
    
    def get_default_piece_positions(self):
        """Get default starting positions for chess pieces"""
        pieces = {}
        
        # Helper function to convert board coordinates to world coordinates
        def board_to_world(col, row):
            x = self.board_offset_x + (col - 3.5) * self.square_size
            y = self.board_offset_y + (row - 3.5) * self.square_size
            z = self.board_height + 0.015  # Half piece height
            return [x, y, z]
        
        # White pieces (rows 0-1)
        for i in range(8):
            pieces[f'white_pawn_{i}'] = board_to_world(i, 1)
        pieces['white_rook_0'] = board_to_world(0, 0)
        pieces['white_rook_1'] = board_to_world(7, 0)
        pieces['white_knight_0'] = board_to_world(1, 0)
        pieces['white_knight_1'] = board_to_world(6, 0)
        pieces['white_bishop_0'] = board_to_world(2, 0)
        pieces['white_bishop_1'] = board_to_world(5, 0)
        pieces['white_queen'] = board_to_world(3, 0)
        pieces['white_king'] = board_to_world(4, 0)
        
        # Black pieces (rows 6-7)
        for i in range(8):
            pieces[f'black_pawn_{i}'] = board_to_world(i, 6)
        pieces['black_rook_0'] = board_to_world(0, 7)
        pieces['black_rook_1'] = board_to_world(7, 7)
        pieces['black_knight_0'] = board_to_world(1, 7)
        pieces['black_knight_1'] = board_to_world(6, 7)
        pieces['black_bishop_0'] = board_to_world(2, 7)
        pieces['black_bishop_1'] = board_to_world(5, 7)
        pieces['black_queen'] = board_to_world(3, 7)
        pieces['black_king'] = board_to_world(4, 7)
        
        return pieces
    
    def publish_chess_visualization(self):
        """Main function to publish all visualization markers"""
        marker_array = MarkerArray()
        
        # Create chessboard
        board_markers, next_id = self.create_chessboard_markers()
        marker_array.markers.extend(board_markers)
        
        # Get piece positions (from TF or default)
        pieces = self.get_piece_positions_from_tf()
        if not pieces:
            # Use default positions if no TF data available
            pieces = self.get_default_piece_positions()
        
        # Create piece markers
        for piece_name, position in pieces.items():
            piece_markers = self.create_piece_marker(piece_name, position, next_id)
            marker_array.markers.extend(piece_markers)
            next_id += 2
        
        # Publish all markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ChessVisualization()
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()