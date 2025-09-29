#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class LaserscanSubscriber(Node):
    def __init__(self):
        super().__init__("laserscan_subscriber")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT) # ros2 topic info /scan -v
        )
    
    def listener_callback(self, msg):
        distances = msg.ranges
        distance_treshold = 1.0 #in meters
        if any(dist < distance_treshold for dist in distances): 
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
