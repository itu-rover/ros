#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin, cos, pi
import random

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class VelocityController(object):
    def __init__(self):
        rospy.init_node('rover_velocity_controller')


        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = random.random()
        self.vy = random.random()
        self.vth = random.random()

        self.current_time =  rospy.Time.now()
        self.last_time =  rospy.Time.now()

        self.odom_pub = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()        
        self.sending_data = None
        self.controller()


    def callback(self, data):
        twist = Twist()
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        print(rospy.get_caller_id() + "  I heard %s", data.data)
        # self.sending_data = "I am listener! %s" % data.data
    
    def controller(self):
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():

            self.current_time = rospy.Time.now()
            # compute odometry in a typical way given the velocities of the robot
            self.dt = (self.current_time - self.last_time).to_sec()
            self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
            self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
            self.delta_th = self.vth * self.dt

            self.x += self.delta_x
            self.y += self.delta_y
            self.th += self.delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                self.odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"

            # set the position
            self.odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))

            # set the velocity
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

            # publish the message


            self.last_time = self.current_time
            rospy.Subscriber('/husky_velocity_controller/cmd_vel', Twist, self.callback)
            # rospy.loginfo(self.odom)
            self.odom_pub.publish(self.odom)
            rate.sleep()


if __name__ == '__main__':
    VelocityController()
