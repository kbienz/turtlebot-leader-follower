#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import rospy
import tf.transformations
from std_msgs.msg import Float32
from std_msgs.msg import Float64
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
        self.max_linear_speed = .108  # Maximum linear speed
        self.MAX_ANG_VEL = 1.2 #Max angular velocity (rad/sec)

#        self.min_safe_distance = 0.05  # Minimum safe distance to leader

        # PID controller gains
        self.linear_kp = .25
        self.linear_ki = .08
        self.linear_kd = .2      #.1
        self.linear_scalar = 1.0 #Linear scaler, used for slowing down linear when making a turn

        self.angular_kp = 0.0025 #0.0014 BEST
        self.angular_kd = 0.007   #0.005 BEST
        self.angular_ki = 0

        # Controller state
        self.linear_error_sum = 0
        self.last_linear_error = 0
        self.last_angular_error = 0
        self.last_time = rospy.Time.now()

        # Robot state
        self.leader_distance = 0.0
        self.leader_angle = 0.0

        #initialize motor command variable
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Publishers and subscribers
        self.sub_lidar = rospy.Subscriber('/scan', LaserScan, self.cbLidar)
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    def cbLidar(self, scan):
        # Process LIDAR data to detect Leader
        self.detect_leader(scan)
        self.follow_leader()

    def cbFollowLane(self, desired_center):
        center = desired_center.data
        error = center - 250
        angular_z = self.angular_kp * error + self.angular_kd * (error - self.last_angular_error)
        self.last_angular_error = error
        error = error * 2 #Turn 500x300 to 1000x600
        if error>=500:
           error = 500
        elif error<=-500:
           error = -500
        print(f"Error: {error}\n")
        self.angular_z = -max(angular_z, -self.MAX_ANG_VEL) if angular_z < 0 else -min(angular_z, self.MAX_ANG_VEL)
        self.linear_scalar = 2*(1-np.abs(error)/500)**2.2  #2.2Initilialize scalar, ranging from 0 to 1.2
        self.linear_scalar =  min(self.linear_scalar, 1) #Set upper limit on scalar = 1
    def detect_leader(self, scan):
        """
        Process LIDAR scan to detect leader
        """
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        # Find index for closest non-zero range lidar value
        min_index = 0
        for i in range(1, len(ranges)):
            if ranges[i] < ranges[min_index] and ranges[i] > .005:
                min_index = i

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
        #print(f"Began PID error calculation")
        linear_error = self.leader_distance - self.desired_distance
        self.error_queue.append(linear_error)
        self.linear_error_sum = sum(self.error_queue) * dt
        linear_error_delta = (linear_error - self.last_linear_error) / dt

        linear_velocity = (self.linear_kp * linear_error +
                           self.linear_ki * self.linear_error_sum +
                           self.linear_kd * linear_error_delta)


        # Update error history
        self.last_linear_error = linear_error

        # Update linear velocity
        self.linear_x = np.clip(linear_velocity, -self.max_linear_speed,
                                  self.max_linear_speed)

    def run(self):
        """
        Main control loop
        """
        rate = rospy.Rate(10)  # 10 Hz control loop

        while not rospy.is_shutdown():
            rate.sleep()
            twist = Twist()

            if self.linear_x < 0: #If backing up, send the reverse turning command
                self.angular_z *= -1

            #print(f"Linear scalar: {self.linear_scalar}\n")
            twist.linear.x = self.linear_x * self.linear_scalar #Set linear velocity
            twist.angular.z = self.angular_z
            self.pub_cmd_vel.publish(twist) #Send out command


if __name__ == '__main__':
    try:
        follower = RobotFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
