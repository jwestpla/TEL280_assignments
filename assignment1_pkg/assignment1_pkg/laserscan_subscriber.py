import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserscanSubscriber(Node):
    def __init__(self):
        super().__init__("laserscan_subscriber")
        self.subscription = self.create_subscription(
            msg_type = LaserScan,
            topic = "/scan",
            callback=self.listener_callback,
            qos_profile= 10#QoSProfile(depth=10, reliability=ReliabiltyProfile.RELIABLE) #??
        )
    
    def listener_callback(self, msg):
        distances = msg.ranges
        if any(dist < 1 for dist in distances): 
            self.get_logger().info('Obstacle')
        else:
            self.get_logger().info('Free')

def main(args=None):
    rclpy.init(args = args)
    node = LaserscanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

