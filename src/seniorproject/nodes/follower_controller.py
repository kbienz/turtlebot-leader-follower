#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import rospy
import tf.transformations
from std_msgs.msg import Float32
from collections import deque

class RobotFollower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_follower', anonymous=True)

        # Store last 60 errors, for integral controller
        self.integral_window_size = 45
        self.error_queue = deque(maxlen=self.integral_window_size)

        # Follower control parameters
        self.desired_distance = .3  # Desired following distance in meters
        self.max_linear_speed = .1  # Maximum linear speed
        self.max_angular_speed = 2.0  # Maximum angular speed
        self.min_safe_distance = 0.05  # Minimum safe distance to leader

        # PID controller gains
        self.linear_kp = .25
        self.linear_ki = .08
        self.linear_kd = .1
        self.angular_kp = .5
        self.angular_ki = 0
        self.angular_kd = 0

        # Controller state
        self.linear_error_sum = 0
        self.angular_error_sum = 0
        self.last_linear_error = 0
        self.last_angular_error = 0
        self.last_time = rospy.Time.now()

        # Robot state
        # self.leader_detected = True
        self.leader_distance = 0.0
        self.leader_angle = 0.0
        self.motors_enabled = True

        # Publishers and subscribers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.linear_x_pub = rospy.Publisher('/linear_x', Float32, queue_size=1)

    def lidar_callback(self, scan):
        # Process LIDAR data to detect Leader
        self.detect_leader(scan)
        self.follow_leader()

    def detect_leader(self, scan):
        """
        Process LIDAR scan to detect leader
        """
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        # Find index for closest non-zero range lidar value
        min_index = 0
        for i in range(1, len(ranges)):
            if ranges[i] < ranges[min_index] and ranges[i] > .01:
                min_index = i
        degree = angles[min_index] * 180 / np.pi
        #print(f"Angle: {round(degree,2)} || Dist: {round(ranges[min_index], 2)}meters")

        # Leader angle and distance
        self.leader_distance = ranges[min_index]
        self.leader_angle = angles[min_index]

    def follow_leader(self):
        """
        Implement PID control for follower
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Linear control
        print(f"Began PID error calculation")
        linear_error = self.leader_distance - self.desired_distance
        self.error_queue.append(linear_error)
        self.linear_error_sum = sum(self.error_queue) * dt
        linear_error_delta = (linear_error - self.last_linear_error) / dt

        linear_velocity = (self.linear_kp * linear_error +
                           self.linear_ki * self.linear_error_sum +
                           self.linear_kd * linear_error_delta)

        # Angular Control
        if self.leader_angle < np.pi:
            angular_error = self.leader_angle

        elif self.leader_angle > np.pi:
            angular_error = -2*np.pi + self.leader_angle

        self.angular_error_sum += angular_error * dt
        angular_error_delta = (angular_error - self.last_angular_error) / dt

        angular_velocity =  (self.angular_kp * angular_error +
                             self.angular_ki * self.angular_error_sum +
                             self.angular_kd * angular_error_delta)
        print(f"Completed PID error Calculation")

        # Update error history
        self.last_linear_error = linear_error
        self.last_angular_error = angular_error

        # Apply velocity limits
        linear_velocity = np.clip(linear_velocity, -self.max_linear_speed,
                                  self.max_linear_speed)
        angular_velocity = np.clip(angular_velocity, -self.max_angular_speed,
                                   self.max_angular_speed)

        # Publish control command
        linear_x = Float32()
        linear_x.data = linear_velocity
        self.linear_x_pub.publish(linear_x)
        print(f"Sent {round(linear_velocity, 2)} to /cmd_vel")

    def run(self):
        """
        Main control loop
        """
        rate = rospy.Rate(20)  # 20 Hz control loop

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        follower = RobotFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
