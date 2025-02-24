import rclpy
from rclpy.node import Node
from multi_agent_mapping.msg import GlobalDescriptor

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Create publisher
        self.pub_descriptors = self.create_publisher(GlobalDescriptor, 
            'a/distributedMapping/globalDescriptors', 10)  # QoS depth of 10

        # Create subscriber
        self.sub_descriptors = self.create_subscription(
            GlobalDescriptor,
            'a/distributedMapping/globalDescriptors',
            self.listener_callback,
            10  # QoS depth of 10
        )

        # Publish a message for testing
        self.publish_message()

    def publish_message(self):
        msg = GlobalDescriptor()
        msg.index = 1  # Example index
        self.get_logger().info(f'Publishing Global Descriptor with index: {msg.index}')
        self.pub_descriptors.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Global Descriptor with index: {msg.index}')

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()