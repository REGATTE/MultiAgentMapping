import rclpy
from rclpy.node import Node
from multi_agent_mapping.msg import GlobalDescriptor

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(GlobalDescriptor, '/b/distributedMapping/globalDescriptors', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every second

    def publish_message(self):
        message = GlobalDescriptor()
        message.index = 1  # Example index
        message.header.stamp = self.get_clock().now().to_msg()  # Get current time
        # Fill in other fields as necessary
        self.publisher_.publish(message)
        self.get_logger().info(f'Published message with index: {message.index}')

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()