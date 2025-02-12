#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32  # Added for publishing distance

class TestLidar:
    def __init__(self):
        # Remove unnecessary parameters since we're only doing LiDAR processing
        self.sub_lidar_data = rospy.Subscriber('/scan_filtered', LaserScan, self.cbFindForwardDistance, queue_size=1)
        # Changed publisher to publish float value instead of compressed image
        self.pub_forward_distance = rospy.Publisher('/forward_distance', Float32, queue_size=1)
        print("Successfuly initialized LIDAR test node!")
        self.previous_distance = 0
        self.forward_distance = 0


    def cbFindForwardDistance(self, lidar_data):
            try:
                num_points = len(lidar_data.ranges)
                #mid_point = num_points//2
                self.forward_distance = min(lidar_data.ranges[0],  lidar_data.ranges[-1], lidar_data.ranges[+1],
                lidar_data.ranges[-2], lidar_data.ranges[+2])
                self.forward_velocity = (self.forward_distance - self.previous_distance)
                if self.forward_distance>0 and not np.isinf(self.forward_distance) and not np.isnan(self.forward_distance):
                    print(f"Dist: {round(self.forward_distance, 2)} || Vel: {round(self.forward_velocity, 2)}")
                    # Publish the distance as a Float32 message
                    msg = Float32()
                    msg.data = self.forward_distance
                    self.pub_forward_distance.publish(msg)
                    self.previous_distance = self.forward_distance
                
            except Exception as e:
                rospy.logerr(f"Error in LiDAR callback: {e}")

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('test_lidar')
    node = TestLidar()  # Fixed class name
    node.main()
