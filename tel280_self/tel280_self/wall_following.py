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

DIST_TARGET = 0.3

class WallFollowing(Node):
    def __init__(self):
        super().__init__('WallFollowing')
        self.lidar = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.pub = self.create_publisher(MarkerArray, "/detected_walls", 10)
        self.move_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.cmd = Twist()
        self.points_xy = []
        self.find_wall = True
        self.drive_along = False
        self.turn = False


    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        self.points_xy = []

        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            theta = angle_min + i * angle_increment
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            self.points_xy.append((x, y))
        lines = self.ransac()

        self.publish_marker(lines)
        
        closest_point, closest_line, distance = self.find_closest_wall(lines)
        if self.find_wall:
            if closest_line:
                self.turn_to_closest_wall(closest_point, distance)
        if self.turn:
            self.turn_90(closest_point, distance)
        if self.drive_along:
            if closest_line:
                self.drive(closest_line, distance)

    
    def timer_callback(self):
        
        self.move_pub.publish(self.cmd)   
    
    def drive(self, line, dist):
        x1,y1,x2,y2 = line

        tx = x2 - x1
        ty = y2 - y1
        theta_wall = math.atan2(ty, tx) - math.pi/2

        if theta_wall > math.pi/2:
            theta_wall -= math.pi
        elif theta_wall < -math.pi/2:
            theta_wall += math.pi
        
        v_max = 0.2
        w_max = 0.3
        e_d = DIST_TARGET - dist
        w = max(theta_wall+e_d, w_max)
        if theta_wall > 0:
            w = -w
        
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = float(w)

    def turn_to_closest_wall(self, closest_point, dist):
        x = closest_point[0]
        y = closest_point[1]
        yaw_err = math.atan2(y, x)

        
        if abs(yaw_err) > 0.1:  
            self.cmd.angular.z = 0.2 * (1 if yaw_err > 0 else -1)
        else:
            self.cmd.angular.z = 0.0 
            if dist > 0.3:
                self.cmd.linear.x = 0.1
            else:
                self.cmd.linear.x = 0.0
                self.find_wall = False
                self.turn = True
                self.drive_along = False
        self.get_logger().info(f"Angle: {yaw_err}, Distance: {dist}")
    
    def turn_90(self, closest_point, dist):
        x = closest_point[0]
        y = closest_point[1]
        yaw_err = math.atan2(y, x) - math.pi/2

        
        if abs(yaw_err) > 0.1:  
            self.cmd.angular.z = 0.2 * (1 if yaw_err > 0 else -1)
        else:
            self.cmd.angular.z = 0.0 
            self.find_wall = False
            self.turn = False
            self.drive_along = True

    def find_closest_wall(self, lines):
        best_dist = 9999
        closest_point = None
        closest_line = None
        
        for i, (x1,y1,x2,y2) in enumerate(lines): 
            dist, point = self.dist_to_segment(x1,y1,x2,y2)
            if dist < best_dist:
                closest_point = point
                closest_line = (x1,y1,x2,y2)
                best_dist = dist
        
        return closest_point, closest_line, best_dist
    
    def dist_to_segment(self, x1,y1,x2,y2):
        p = np.array([x1,y1])
        v = np.array([x2-x1, y2-y1])
        denom = np.dot(v,v)
        if denom == 0:
            # endepunktene er samme punkt
            return np.linalg.norm(p)
        t = -np.dot(p,v) / denom
        t = max(0.0, min(1.0, t))
        closest = p + t*v
        return np.linalg.norm(closest), closest
    
    def publish_marker(self, lines):
        marker_array = MarkerArray()
        for i, (x1,y1,x2,y2) in enumerate(lines): 
            marker = Marker()
            marker.header.frame_id = "base_scan"
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.g, marker.color.a = 1.0, 1.0

            p1 = Point(x=x1, y=y1, z=0.0)
            p2 = Point(x=x2, y=y2, z=0.0)
            marker.points = [p1, p2]
            marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()

            marker_array.markers.append(marker)

        self.pub.publish(marker_array)
    
    def ransac(self):
        if len(self.points_xy) < 2:
            return []

        # Lag arrays
        pts = np.array(self.points_xy)
        X = pts[:, 0].reshape(-1, 1)   
        y = pts[:, 1] 

        finding_walls = True   
        list_of_lines = []    
        while finding_walls:
            # RANSAC line fit
            ransac = linear_model.RANSACRegressor(
                linear_model.LinearRegression(),
                residual_threshold=0.1, 
                min_samples=2
            )
            ransac.fit(X, y)

            inlier_mask = ransac.inlier_mask_
            outlier_mask = np.logical_not(inlier_mask)

            # Få koeffisienter for linjen y = a*x + b
            a = ransac.estimator_.coef_[0]
            b = ransac.estimator_.intercept_

            # Finn linjesegment basert på inliers (min/max x)
            x_inliers = X[inlier_mask]
            y_inliers = y[inlier_mask]
            x_min, x_max = np.min(x_inliers), np.max(x_inliers)
            y_min = a * x_min + b
            y_max = a * x_max + b

            inlier_pts = pts[inlier_mask]                   
            pts = pts[~inlier_mask] 
            X = pts[:, 0].reshape(-1, 1)   
            y = pts[:, 1] 

            start = time.time()
            p1, p2, keep = self.prune_line_by_gap(inlier_pts, gap_thresh=1, min_len=0.3, min_pts=10, reduce=True)
            end = time.time()
            print(f"Kjøretid på prune: {end - start:.4f} sekunder")
            if p1 is None:
                pass
            else:
                # bruk segmentet (p1, p2) til RViz
                list_of_lines.append((float(p1[0]), float(p1[1]), float(p2[0]), float(p2[1])))
                finding_walls = False

        return list_of_lines

    def prune_line_by_gap(self, inlier_pts, gap_thresh=0.12, min_len=0.8, min_pts=15, reduce=True):
        if len(inlier_pts) < 2: 
            return None, None, None

        # linjeretning via total least squares (stabilt også for vertikale linjer)
        c = inlier_pts.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_pts - c)
        d = vh[0] / np.linalg.norm(vh[0])      # line_dir (2,)

        # projisér punkter langs linja og sorter
        s = (inlier_pts - c) @ d               # (N,)
        order = np.argsort(s)
        s_sorted = s[order]

        gaps = np.diff(s_sorted)
        has_gap = np.any(gaps > gap_thresh)

        if has_gap and not reduce:
            return None, None, None  # kast hele linja

        # del på hull og velg beste del (flest punkter, ties→størst utstrekning)
        cuts = np.where(gaps > gap_thresh)[0]
        clusters = np.split(order, cuts + 1) if has_gap else [order]
        def key(idx): 
            return (len(idx), s[idx].max() - s[idx].min())
        best = max(clusters, key=key)

        if len(best) < min_pts:
            return None, None, None

        s_min, s_max = s[best].min(), s[best].max()
        p1, p2 = c + s_min * d, c + s_max * d
        if np.linalg.norm(p2 - p1) < min_len:
            return None, None, None

        keep_mask = np.zeros(len(inlier_pts), dtype=bool)
        keep_mask[best] = True
        return p1, p2, keep_mask


def main(args=None):
    rclpy.init(args = args)
    node = WallFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()