#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import numpy as np 
from sklearn import datasets, linear_model
from sklearn.cluster import DBSCAN
from collections import deque 
import time

DIST_WALL = 0.3
LEFT = True

class WallFollowing(Node):
    def __init__(self):
        super().__init__('WallFollowing')

        #Subsciber to laserscan
        self.lidar = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.move_pub = self.create_publisher(Twist, "/cmd_vel", 10)                  # cmd_vel publisher
        self.marker_pub = self.create_publisher(MarkerArray, "/detected_walls", 10)   # marker publisher for Rviz
        self.timer_ = self.create_timer(0.1, self.timer_callback)                     # Timer to publish msgs to topics
        self.cmd_vel = Twist()

        #Statevaribles
        self.find_wall_state = False
        self.turn_state = False
        self.drive_state = False

        #Wall
        self.line = []

    def scan_callback(self, msg):
        """
        This function converts the ranges from laserscan to (x, y) coordinates.
        It then calls the ransac function to find the best wall and stores it in self.line
        """
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        points = []
        
        # convert from polar to (x,y)
        for i, r in range(ranges):
            if not math.isfinite(r):
                continue
            theta = angle_min + i * angle_increment
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            points.append((x,y))
        
        self.line = self.ransac(points)

    def timer_callback(self):
        if self.line: #Checks if a line has been found
            self.publish_marker() #Publishes the lie to Rviz

            closest_point = self.get_closest_point(self.line)
            if self.find_wall_state:
                self.turn_to_closest_wall(closest_point)
            if self.turn_state:
                self.turn_parallel(closest_point)
            if self.drive_state:
                self.drive_along(closest_point)

    def turn_to_closest_wall(self, point):
        yaw_err = self.get_angle(point)
        dist = self.get_angle(point)

        if abs(yaw_err) > 0.1:  
            self.cmd.angular.z = 0.2 * (1 if yaw_err > 0 else -1)
        else:
            self.cmd.angular.z = 0.0 
            if dist > DIST_WALL:
                self.cmd.linear.x = 0.1
            else:
                self.cmd.linear.x = 0.0
                self.find_wall_state = False
                self.turn_state= True
                self.drive_state = False

    def turn_parallel(self, point):
        #Rotate the robot parallel to the wall
        angle_err = self.get_angle(point) + (np.pi/2 if LEFT else -np.pi/2)

        if abs(angle_err) > 0.1:
            self.cmd_vel.angular.z = 0.2 * (1 if angle_err > 0 else -1) # Turns the robot
        else:
            self.cmd_vel.angular.z = 0.0
            self.find_wall_state = False
            self.turn_state = False
            self.drive_state = True

    def drive_along(self, point):
        dist = self.get_distance(point)
        theta_wall = self.get_angle(point) + (np.pi/2 if LEFT else -np.pi/2)
        if theta_wall > math.pi/2:
            theta_wall -= math.pi
        elif theta_wall < -math.pi/2:
            theta_wall += math.pi
        
        v_max = 0.2
        w_max = 0.3
        e_d = DIST_WALL - dist
        w = max(theta_wall+e_d, w_max)
        if theta_wall > 0:
            w = -w
        
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = float(w)

    def get_closest_point(self, line):
        #Return the closest point to the robot on the line
        x1, y1 = line[0]
        x2, y2 = line[1]
        
        p = np.array([x1, y1])
        v = np.array([x2 - x1, y2 - y1])
        v_2 = np.dot(v,v)
        
        t = -np.dot(p,v) / v_2 
        t = max(0.0, min(1.0, t)) #keeps t between 0 and 1
        closest_point = p + t * v
        return closest_point

    def get_angle(self, point):
        #Returns the angle to the point
        return math.atan2(point[0], point[1])
    
    def get_distance(self, point):
        #Returns a the distance to the point
        return np.linalg.norm(point)

    def ransac(self, points):
        """
        Gets a list of points and returns the best line in the pointcloud using ransac
        """
        pts = np.array(points)
        X = pts[:, 0].reshape(-1,1)
        y = pts[:, 1]

        ransac = linear_model.RANSACRegressor(
            linear_model.LinearRegression(),
            residual_threshold=0.1,
            min_samples=2
        ) 

        ransac.fit(X, y)
        
        #Gets the points after the ransac analysis
        inliner_mask = ransac.inlier_mask_
        x_inliners = X[inliner_mask]

        a = ransac.estimator_.coef_[0] 
        b = ransac.estimator_.intercept_
        
        x1, x2 = np.min(x_inliners), np.max(x_inliners)
        y1, y2 = a*x1 +b, a*x2 + b

        return [(x1, y1),(x2, y2)] 


    def publish_marker(self, line):
        x1, y1 = line[0]
        x2, y2 = line[1]
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.g, marker.color.a = 1.0, 1.0
        p1 = Point(x=x1, y=y1, z=0.0)
        p2 = Point(x=x2, y=y2, z=0.0)
        marker.points = [p1, p2]
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
        marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args = args)
    node = WallFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()