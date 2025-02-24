#!/usr/bin/env python3
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import rospy
import tf.transformations

class RobotFollower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_follower', anonymous=True)

        # Robot B control parameters
        self.desired_distance = .2  # Desired following distance in meters
        self.max_linear_speed = 3.0  # Maximum linear speed
        self.max_angular_speed = 3.0  # Maximum angular speed
        self.min_safe_distance = 0.04  # Minimum safe distance to leader

        # PID controller gains
        self.linear_kp = .5
        self.linear_ki = 0
        self.linear_kd = 0
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
        self.leader_detected = True
        self.leader_distance = 0.0
        self.leader_angle = 0.0
        self.motors_enabled = True
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    def lidar_callback(self, scan):
        # Process LIDAR data to detect Robot A
        self.detect_leader(scan)
        self.follow_leader()

    def detect_leader(self, scan):
        """
        Process LIDAR scan to detect Robot A
        Uses a simple clustering approach to identify robot-sized objects
        """
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        min_index = 0
        for i in range(1, len(ranges)):
            if ranges[i] < ranges[min_index] and ranges[i] > .01:
                min_index = i
        degree = angles[min_index] * 180 / np.pi
        print(f"Angle: {round(degree,2)} || Dist: {round(ranges[min_index], 2)}meters")
       # print(f"Nearest Distance: {round(ranges[min_index], 2)}")
        self.leader_distance = ranges[min_index]
        self.leader_angle = angles[min_index]
#        print(f"Angles: {angles}")

        # Filter out invalid readings and apply distance threshold
        #valid_indices = np.where((ranges > scan.range_min) &
                               #(ranges < scan.range_max))[0]

        #if len(valid_indices) == 0:
           # self.leader_detected = False
           # return

        # Simple clustering to find robot-sized objects
        #clusters = []
        #current_cluster = [valid_indices[0]]

        #for i in range(1, len(valid_indices)):
            #if valid_indices[i] - valid_indices[i-1] <= 3:  # Adjacent points
                #current_cluster.append(valid_indices[i])
            #else:
                #if len(current_cluster) >= 5:  # Minimum cluster size
                    #clusters.append(current_cluster)
                #current_cluster = [valid_indices[i]]

        #if len(current_cluster) >= 5:
            #clusters.append(current_cluster)

        # Find the most likely cluster (Robot A)
        #best_cluster = None
        #min_distance = float('inf')

        #for cluster in clusters:
            #cluster_distance = np.mean(ranges[cluster])
            #if cluster_distance < min_distance:
               # min_distance = cluster_distance
               # best_cluster = cluster

        #if best_cluster is not None:
            # Calculate leader position
            #self.leader_distance = np.mean(ranges[best_cluster])
            #self.leader_angle = np.mean(angles[best_cluster])
            #self.leader_detected = True
        #else:
            #self.leader_detected = False

    def follow_leader(self):
        """
        Implement PID control for following Robot A
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Linear control
        linear_error = self.leader_distance - self.desired_distance
        self.linear_error_sum += linear_error * dt
        linear_error_delta = (linear_error - self.last_linear_error) / dt

        linear_velocity = (self.linear_kp * linear_error +
                           self.linear_ki * self.linear_error_sum +
                           self.linear_kd * linear_error_delta)

        if self.leader_angle < np.pi:
            angular_error = self.leader_angle

        elif self.leader_angle > np.pi:
            angular_error = -2*np.pi + self.leader_angle

        self.angular_error_sum += angular_error * dt
        angular_error_delta = (angular_error - self.last_angular_error) / dt

        angular_velocity =  (self.angular_kp * angular_error +
                             self.angular_ki * self.angular_error_sum +
                             self.angular_kd * angular_error_delta)

        angular_degrees = angular_error * 180/np.pi
        print(f"Angular Error: {round(angular_degrees,2)}")

        # Update error history
        self.last_linear_error = linear_error
        self.last_angular_error = angular_error

        # Apply velocity limits
        linear_velocity = np.clip(linear_velocity, -self.max_linear_speed,
                                  self.max_linear_speed)
        angular_velocity = np.clip(angular_velocity, -self.max_angular_speed,
                                   self.max_angular_speed)

        # Safety check
        if self.leader_distance < self.min_safe_distance:
            linear_velocity = min(0, linear_velocity)  # Only allow backing up
            print("[WARN], Dist<Safe_Dist, Only Backing Up is Enabled")

        # Publish control command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        if self.motors_enabled == True:
            self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        """
        Main control loop
        """
        rate = rospy.Rate(20)  # 20 Hz control loop

        while not rospy.is_shutdown():
            if not self.leader_detected:
                # Stop if leader is lost
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = RobotFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
