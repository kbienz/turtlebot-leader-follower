#!/usr/bin/env python3

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VelocityConverter:
    def __init__(self):
        rospy.init_node('velocity_converter')
        self.vx = rospy.get_param("~vx", .2)
        self.vy = rospy.get_param("~vy", .3)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.yaw = 0
        self.rate = rospy.Rate(10)

    def odom_callback(self,msg):
        orientation_q = msg.pose.pose.orientation
        _,_, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])


    def run(self):
        while not rospy.is_shutdown():
            print("hello world")
            vx_robot = math.cos(self.yaw) * self.vx + math.sin(self.yaw) * self.vy
            vy_robot = -math.sin(self.yaw) * self.vx + math.cos(self.yaw) * self.vy
            twist = Twist()
            twist.linear.x = vx_robot
            twist.angular.z = math.atan2(vy_robot, vx_robot) if abs(vx_robot) > .01 else 0.0

            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__== '__main__':
    try:
        VelocityConverter().run()
    except rospy.ROSInterruptException:
        pass
