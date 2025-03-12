#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class LidarClusterDetector:
    def __init__(self):
        self.sub_lidar_data = rospy.Subscriber('/scan_filtered', LaserScan, self.cb_cluster_objects, queue_size=1)
        self.pub_nearest_distance = rospy.Publisher('/nearest_distance', Float32, queue_size=1)

        # Clustering parameters
        self.distance_threshold = 0.04  # Maximum distance between points in a cluster (in meters)
        self.min_cluster_size = 1  # Minimum points to form a cluster

        self.previous_distance = 0
        self.nearest_distance = 0

    def polar_to_cartesian(self, ranges, angles):
        """Convert polar coordinates to cartesian"""
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return x, y

    def simple_clustering(self, points_x, points_y):
        """Simple distance-based clustering algorithm"""
        n_points = len(points_x)
        if n_points == 0:
            return []

        # Initialize clusters
        clusters = []
        visited = [False] * n_points

        # For each point
        for i in range(n_points):
            if visited[i]:
                continue

            # Start a new cluster
            cluster = []
            self.expand_cluster(i, points_x, points_y, visited, cluster)
            
            # Only keep clusters with enough points
            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)
                
        return clusters
        
    def expand_cluster(self, point_idx, points_x, points_y, visited, cluster):
        """Expand cluster from seed point"""
        # Add point to cluster
        cluster.append((points_x[point_idx], points_y[point_idx]))
        visited[point_idx] = True
        
        # Look for neighbors
        for j in range(len(points_x)):
            if visited[j]:
                continue
                
            # Calculate distance
            dist = math.sqrt((points_x[point_idx] - points_x[j])**2 + 
                             (points_y[point_idx] - points_y[j])**2)
                             
            if dist <= self.distance_threshold:
                self.expand_cluster(j, points_x, points_y, visited, cluster)
        
    def cb_cluster_objects(self, lidar_data):
        try:
            max_range = 1.5
            # Create angle array from scan parameters
            angles = np.arange(
                lidar_data.angle_min,
                lidar_data.angle_max + lidar_data.angle_increment,
                lidar_data.angle_increment
            )[:len(lidar_data.ranges)]
            
            # Filter out invalid readings (inf, nan, zero)
            valid_indices = np.isfinite(lidar_data.ranges) & (np.array(lidar_data.ranges) > 0) & (np.array(lidar_data.ranges) <= max_range)	
            valid_ranges = np.array(lidar_data.ranges)[valid_indices]
            valid_angles = angles[valid_indices]
            
            if len(valid_ranges) < self.min_cluster_size:
                rospy.logwarn("Not enough valid LiDAR points for clustering")
                return
                
            # Convert to cartesian coordinates
            points_x, points_y = self.polar_to_cartesian(valid_ranges, valid_angles)
            
            # Perform simple clustering
            clusters = self.simple_clustering(points_x, points_y)
            
            # If no clusters found
            if not clusters:
                rospy.loginfo("No clusters detected")
                return
                
            # Find the nearest cluster
            nearest_distance = float('inf')
            nearest_cluster = None
            
            for i, cluster in enumerate(clusters):
                # Calculate cluster centroid
                centroid_x = np.mean([p[0] for p in cluster])
                centroid_y = np.mean([p[1] for p in cluster])
                
                angle_rad = math.atan2(centroid_y, centroid_x)
                angle_deg = math.degrees(angle_rad)
                # Calculate distance to centroid
                centroid_distance = math.sqrt(centroid_x**2 + centroid_y**2)
                
                if centroid_distance < nearest_distance:
                    nearest_distance = centroid_distance
                    nearest_cluster = i
            
            if nearest_cluster is not None:
                self.nearest_distance = nearest_distance
                velocity = self.nearest_distance - self.previous_distance
                
                # Publish results
#                msg = Float32()
#                msg.data = self.nearest_distance
#                self.pub_nearest_distance.publish(msg)
                
                self.previous_distance = self.nearest_distance
                
                # Log information about the nearest cluster
                #rospy.loginfo(f"Nearest cluster ({nearest_cluster}) - Distance: {round(self.nearest_distance, 2)}m, Points: {len(clusters[nearest_cluster])}, Velocity: {round(velocity, 2)}m/s")
                #rospy.loginfo(f"Nearest cluster ({nearest_cluster}) - Distance: {round(self.nearest_distance, 2)}m, Angle: {round(angle_deg, 2)}°, Points: {len(clusters[nearest_cluster])}, Velocity: {round(velocity, 2)}m/s")
                
                robot = []
                rospy.loginfo(f"Found {len(clusters)} clusters:")
                for i, cluster in enumerate(clusters):
                    centroid_x = np.mean([p[0] for p in cluster])
                    centroid_y = np.mean([p[1] for p in cluster])
                    distance = math.sqrt(centroid_x**2 + centroid_y**2)
                    angle_rad = math.atan2(centroid_y, centroid_x)
                    angle_deg = math.degrees(angle_rad)
                    total_angle = lidar_data.angle_increment*len(cluster)
                    arc_len = total_angle * distance
#                    initial_angle = np.arctan2(cluster[0][1], cluster[0][0])
#                    final_angle = np.arctan2(cluster[-1][1], cluster[-1][0])
#                    arc_len = (final_angle - initial_angle) * distance
                    if .05 <= arc_len <= .17:
                        robot.append(distance)
                # robot approximate arc length range from .14 to .21
                    #rospy.loginfo(f"  Cluster {i}: Points: {len(cluster)}, Distance: {round(distance, 2)}m, Angle: {round(angle_deg, 2)}°, Arc Length: {round(arc_len,2)}, Robot:{robot}")
                print(round(min(robot), 3))
          
                msg = Float32()
                msg.data = min(robot)
                self.pub_nearest_distance.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error in LiDAR clustering callback: {e}")

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lidar_cluster_detector')
    node = LidarClusterDetector()
    node.main()
