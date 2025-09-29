#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LaserScanNode(Node):
    def __init__(self):
        super().__init__('LaserScan')
        self.lidar = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        if min(ranges) < 1.0:
            self.get_logger().info('Obstacle')
        else:
            self.get_logger().info('Free')

def main(args=None):
    rclpy.init(args = args)
    node = LaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()