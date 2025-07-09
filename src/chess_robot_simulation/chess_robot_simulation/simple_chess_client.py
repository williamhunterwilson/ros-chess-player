#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import threading

class SimpleChessClient(Node):
    def __init__(self):
        super().__init__('simple_chess_client')
        
        # Publisher for chess moves
        self.move_pub = self.create_publisher(String, '/chess_move_command', 10)
        
        self.get_logger().info('Simple Chess Client started')
        self.get_logger().info('Enter chess moves in format: e2e4')
        self.get_logger().info('Type "quit" to exit')
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
    
    def input_loop(self):
        """Loop to get user input"""
        while rclpy.ok():
            try:
                move = input("Enter move (e.g., e2e4): ").strip()
                
                if move.lower() == 'quit':
                    self.get_logger().info('Exiting...')
                    rclpy.shutdown()
                    break
                
                if self.validate_move(move):
                    msg = String()
                    msg.data = move
                    self.move_pub.publish(msg)
                    self.get_logger().info(f'Published move: {move}')
                else:
                    print(f"Invalid move format: {move}")
                    print("Please use format like: e2e4, d7d5, etc.")
                    
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f'Error reading input: {e}')
    
    def validate_move(self, move):
        """Basic validation of chess move notation"""
        if len(move) != 4:
            return False
        
        if move[0] not in 'abcdefgh' or move[2] not in 'abcdefgh':
            return False
        
        if move[1] not in '12345678' or move[3] not in '12345678':
            return False
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleChessClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()