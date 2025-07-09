#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import chess
import sys
import re

class ChessMoveClient(Node):
    def __init__(self):
        super().__init__('chess_move_client')
        
        # Publisher for sending moves as String messages
        self.move_publisher = self.create_publisher(String, 'chess_move_cmd', 10)
        
        # Client for calling the move service
        self.move_client = self.create_client(AddTwoInts, 'make_chess_move')
        
        # Wait for the service to be available
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for chess move service...')
        
        self.get_logger().info('Chess move client ready! Enter moves in UCI format (e.g., "e2e4"):')

    def send_move_cmd(self, move_str):
        # Validate the move format
        if not self.is_valid_move_format(move_str):
            self.get_logger().error(f'Invalid move format: {move_str}')
            self.get_logger().info('Moves should be in UCI format, e.g., "e2e4" or "g1f3"')
            return False
        
        # Convert move to service request format
        from_square = self.algebraic_to_idx(move_str[0:2]) 
        to_square = self.algebraic_to_idx(move_str[2:4])
        
        # Call the service
        request = AddTwoInts.Request()
        request.a = from_square
        request.b = to_square
        
        self.get_logger().info(f'Sending move: {move_str}')
        
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            success = bool(future.result().sum)
            if success:
                self.get_logger().info('Move executed successfully')
            else:
                self.get_logger().error('Move execution failed')
            return success
        else:
            self.get_logger().error('Service call failed')
            return False

    def is_valid_move_format(self, move_str):
        # Check if the move is in UCI format (e.g., "e2e4")
        pattern = r'^[a-h][1-8][a-h][1-8]$'
        return bool(re.match(pattern, move_str))

    def algebraic_to_idx(self, algebraic):
        # Convert algebraic notation (e.g., "e2") to index (0-63)
        file_str, rank_str = algebraic[0], algebraic[1]
        file_idx = ord(file_str) - ord('a')  # 'a' -> 0, 'h' -> 7
        rank_idx = int(rank_str) - 1         # '1' -> 0, '8' -> 7
        return rank_idx * 8 + file_idx


def main(args=None):
    rclpy.init(args=args)
    client = ChessMoveClient()
    
    if len(sys.argv) > 1:
        # If move is provided as command line argument, send it and exit
        move = sys.argv[1]
        client.send_move_cmd(move)
    else:
        # Interactive mode
        try:
            while True:
                move = input('\nEnter chess move (or q to quit): ').strip()
                if move.lower() in ('q', 'quit', 'exit'):
                    break
                if move:
                    client.send_move_cmd(move)
        except KeyboardInterrupt:
            print('\nExiting...')
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
