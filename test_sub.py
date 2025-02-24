import rclpy
from rclpy.node import Node
from multi_agent_mapping.msg import GlobalDescriptor

class GlobalDescriptorSubscriber(Node):
    def __init__(self):
        super().__init__('global_descriptor_subscriber')
        self.subscription = self.create_subscription(
            GlobalDescriptor,
            'a/distributedMapping/globalDescriptors',  # Replace 'robot_name' with the actual robot name
            self.listener_callback,
            50  # QoS depth
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Global Descriptor with index: {msg.index}')

def main(args=None):
    rclpy.init(args=args)
    global_descriptor_subscriber = GlobalDescriptorSubscriber()
    rclpy.spin(global_descriptor_subscriber)

    # Destroy the node explicitly
    global_descriptor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()