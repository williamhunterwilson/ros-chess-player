import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'chess_move', 10)
        self.get_logger().info(f'Created publisher on topic: {self.publisher.topic_name}')
        self.timer = self.create_timer(2.0, self.publish_message)
        self.count = 0
        
    def publish_message(self):
        msg = String()
        moves = ['e2e4', 'd2d4', 'g1f3', 'b1c3']
        msg.data = moves[self.count % len(moves)]
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()