import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalChessRobotNode(Node):
    def __init__(self):
        super().__init__('chess_robot_node')
        
        # Create a simple subscriber
        self.chess_move_subscriber = self.create_subscription(
            String,
            'chess_move',
            self.chess_move_callback,
            10)
        
        self.get_logger().info('Minimal Chess Robot Node started')
        self.get_logger().info(f'Subscribed to topic: {self.chess_move_subscriber.topic_name}')

    def chess_move_callback(self, msg):
        self.get_logger().info(f'Received chess move: {msg.data}')

def main():
    rclpy.init()
    node = MinimalChessRobotNode()
    try:
        node.get_logger().info('Node running - waiting for messages')
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception occurred: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()