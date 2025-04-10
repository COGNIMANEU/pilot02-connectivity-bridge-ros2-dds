import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloSubscriber(Node):
    def __init__(self):
        super().__init__('hello_subscriber')
        self.subscription = self.create_subscription(String, 'hello', self.listener_callback, 10)
        self.start_time = self.get_clock().now()

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    node = HelloSubscriber()
    end_time = node.get_clock().now() + rclpy.time.Duration(seconds=20)
    while rclpy.ok() and node.get_clock().now() < end_time:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
