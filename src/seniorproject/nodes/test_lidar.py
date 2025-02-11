#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32  # Added for publishing distance

class TestLidar:
    def __init__(self):
        # Remove unnecessary parameters since we're only doing LiDAR processing
        self.sub_lidar_data = rospy.Subscriber('/scan', LaserScan, self.cbFindForwardDistance, queue_size=1)
        # Changed publisher to publish float value instead of compressed image
        self.pub_forward_distance = rospy.Publisher('/forward_distance', Float32, queue_size=1)
        print("1 Successfuly initialized LIDAR test node!")

    def cbFindForwardDistance(self, lidar_data):
        try:
            # Access the middle element safelyy and handle potential inf/nan values
            num_points = int(6.28//lidar_data.angle_increment)
            forward_distance = lidar_data.ranges[0]
            if forward_distance>0 and not np.isinf(forward_distance) and not np.isnan(forward_distance):
                print(f"Forward distance: {round(forward_distance, 3)} Meters | Points Collected: {num_points}")
                # Publish the distance as a Float32 message
                msg = Float32()
                msg.data = forward_distance
                self.pub_forward_distance.publish(msg)
        except IndexError:
            rospy.logwarn("Index error")

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('test_lidar')
    node = TestLidar()  # Fixed class name
    node.main()
