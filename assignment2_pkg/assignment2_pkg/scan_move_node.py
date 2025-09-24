import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follower")

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.publisher = self.create_publisher(
            Twist, 
            '/cmd_vel',
            10      # Amount of messages stored in "queue" when publishing
        )

        self.turning_until = None   # commit-to-turn timestamp

    
    def listener_callback(self, msg):
        twist = Twist()
        desired = 0.3   # target distance to maintain from left wall
        now = self.get_clock().now().nanoseconds / 1e9  # current time in seconds

        # ======================================================
        # 1. HANDLE COMMITTED RIGHT TURN (corner escape)
        # ======================================================
        # If we previously decided to turn right (front+left blocked),
        # keep turning right until the commit time expires.
        if self.turning_until and now < self.turning_until:
            twist.linear.x = 0.0
            twist.angular.z = -0.5   # turn right in place
            self.publisher.publish(twist)
            return
        else:
            self.turning_until = None  # reset once time is up

        # ======================================================
        # 2. EXTRACT LIDAR SECTORS
        # ======================================================
        # Front sector: combine last few and first few indices (~±5°)
        front_vals = [d for d in (msg.ranges[355:] + msg.ranges[:5])
                    if 0.0 < d < msg.range_max]
        f = sum(front_vals) / len(front_vals) if front_vals else None

        # Left side slices
        a_vals = [d for d in msg.ranges[55:65] if 0.0 < d < msg.range_max]  # ~60°
        b_vals = [d for d in msg.ranges[85:95] if 0.0 < d < msg.range_max]  # ~90°

        # ======================================================
        # 3. FRONT BLOCKED LOGIC
        # ======================================================
        if f is not None and f < desired:
            # If front blocked, check if left is also blocked
            b = (sum(b_vals) / len(b_vals)) if b_vals else None
            if b is not None and b < desired:
                # Both front and left blocked -> dead corner
                # Commit to turning right for 2 seconds
                self.get_logger().info(
                    f"Front {f:.2f} m & Left {b:.2f} m -> committing RIGHT turn"
                )
                self.turning_until = now + 2.0
                twist.linear.x = 0.0
                twist.angular.z = -0.5
                self.publisher.publish(twist)
                return
            else:
                # Only front blocked -> turn left in place
                self.get_logger().info(f"Front {f:.2f} m -> turning LEFT")
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                self.publisher.publish(twist)
                return

        # ======================================================
        # 4. LOST WALL CASE
        # ======================================================
        if not a_vals or not b_vals:
            # No reliable readings on the left side
            # Strategy: creep forward slowly while arcing left
            self.get_logger().info("Lost left wall -> creeping forward + turning left")
            twist.linear.x = 0.05
            twist.angular.z = 0.3
            self.publisher.publish(twist)
            return

        # ======================================================
        # 5. NORMAL LEFT-WALL FOLLOWING
        # ======================================================
        # Compute averages for the two beams
        a = sum(a_vals) / len(a_vals)
        b = sum(b_vals) / len(b_vals)

        # Estimate robot orientation relative to wall
        theta = math.radians(30)  # angle difference between a and b sectors
        alpha = math.atan((a*math.cos(theta) - b) / (a*math.sin(theta)))
        dist = b * math.cos(alpha)  # perpendicular distance to wall

        self.get_logger().info(
            f"a(60°)={a:.2f}, b(90°)={b:.2f}, "
            f"alpha={math.degrees(alpha):.1f}°, dist={dist:.2f}"
        )

        # Control law: proportional on distance + angle
        error = dist - desired
        k_dist = 1.0
        k_angle = 0.5

        twist.linear.x = 0.1          # forward speed
        twist.angular.z = k_dist*error + k_angle*alpha
        self.publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
     