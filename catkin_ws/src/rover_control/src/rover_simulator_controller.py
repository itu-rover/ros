#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin,cos, pi
import random
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from geographic_msgs.msg import WayPoint, GeoPoint
import numpy as np

class VelocityController(object):
    def __init__(self):
        rospy.init_node('rover_simulator_controller')


        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        self.current_time =  rospy.Time.now()
        self.last_time =  rospy.Time.now()

        self.odom_pub = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size = 50)
        self.odom_combined_pub = rospy.Publisher('/odom_combined', PoseWithCovarianceStamped, queue_size = 50)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size = 50)
        self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size = 50)
        self.waypoint_pub = rospy.Publisher('/waypoint', WayPoint, queue_size = 50)
        # self.odom_broadcaster = tf.TransformBroadcaster()        
        self.sending_data = None
        self.odom_quat =[0,0,0,1]
        self.twist = Twist()
        self.gps_odom = Odometry()
        self.controller()

    def callback(self,data):

        # Write these values to motors

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.twist.linear.x = data.linear.x 
        self.twist.linear.y = data.linear.y 
        self.twist.linear.z = data.linear.z
        self.twist.angular.x = data.angular.x
        self.twist.angular.y = data.angular.y
        self.twist.angular.z = data.angular.z
        if self.twist.angular.z < 0:
            self.angular_way = "0"
            self.rover_angular_speed = -self.twist.angular.z
        else:
            self.angular_way = "1"
            self.rover_angular_speed = self.twist.angular.z

        # Linear Speed
        self.rover_linear_speed = int(self.twist.linear.x*99/0.5)
        if self.rover_linear_speed < 10:
            self.rover_linear_speed_str = "0" + str(self.rover_linear_speed)
        else:
            self.rover_linear_speed_str = str(self.rover_linear_speed)

        # Angular Speed
        self.rover_angular_speed = int(self.rover_angular_speed*99)
        if self.rover_angular_speed < 10:
            self.rover_angular_speed_str = "0" + str(self.rover_angular_speed)
        else:
            self.rover_angular_speed_str = str(self.rover_angular_speed)

        self.rover_cmd_vel = "B" + "1" + self.rover_linear_speed_str + self.angular_way + self.rover_angular_speed_str + "E"
        print(rospy.get_caller_id() + "  writing %s", self.rover_cmd_vel )
        # self.sending_data = "I am listener! %s" % data.data
    
    def gps_callback(self,data):
            # self.x = data.x
            # self.y = data.y
            self.odom.pose.pose = data.pose.pose 
            # self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)


    def controller(self):
        rate = rospy.Rate(10) # 10hz

        # self.x = self.polar2cartesian([earth_radius, 41.104444, 29.026997])[0] # update self.x everytime
        # self.y = self.polar2cartesian([earth_radius, 41.104444, 29.026997])[1] # update self.x everytime


        while not rospy.is_shutdown():

            # for simulating
            self.vx = self.twist.linear.x
            self.vy = self.twist.linear.y
            self.vth = self.twist.angular.z

            self.current_time = rospy.Time.now()

            # We do not need this part, we are doing our own localization
            # compute odometry in a typical way given the velocities of the robot
            self.dt = (self.current_time - self.last_time).to_sec()
            self.delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
            self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
            self.delta_th = self.vth * self.dt

            # from GPS
            # self.target_location = [earth_radius, 41.103465, 29.027909] # publish to move_base_simple/goal
            # from magnetometer
            # self.yaw = 0

            # X, Y = current location
            # th = current yaw
            """
            self.x += self.delta_x
            self.y += self.delta_y
            self.th += self.delta_th
            """
            # since all odometry is 6DOF we'll need a quaternion created from yaw
            self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # first, we'll publish the transform over tf
            """
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                self.odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )
            """
            # next, we'll publish the odometry message over ROS
            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"

            # set the position
            self.odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))

            # Write a tranform formula for calculating linear.x linear.y and angular.z speed
            # set the velocity
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

            # publish the PoseWithCovarianceStamped
            self.pwcs = PoseWithCovarianceStamped()
            self.pwcs.header.stamp = self.odom.header.stamp
            self.pwcs.header.frame_id = "odom"
            self.pwcs.pose.pose = self.odom.pose.pose
            self.last_time = self.current_time

            # publish the NavSatFix
            self.sensor_data = None
            self.gps_fix = NavSatFix()
            self.gps_fix.header.frame_id = "base_link"
            self.gps_fix.header.stamp = self.odom.header.stamp
            self.gps_fix.status.status = 0 # GPS FIXED
            self.gps_fix.status.service = 1 # GPS SERVICE = GPS
            # Buralar bizden gelecek
            self.gps_fix.latitude = 41.24600
            self.gps_fix.longitude = 29.123123
            self.gps_fix.altitude = 0
            self.gps_fix.position_covariance = [0,0,0,0,0,0,0,0,0]
            self.gps_fix.position_covariance_type = 0

            # publish the NavSatFix
            self.waypoint = WayPoint()
            # self.waypoint.id.uuid = [1]
            self.waypoint.position.latitude = 41.24678
            self.waypoint.position.longitude = 29.123100
            self.waypoint.position.altitude = 0
            # self.waypoint.props.key = "key"
            # self.waypoint.props.value = "1"

            # publish the NavSatFix
            self.imuMsg = Imu()
            self.imuMsg.orientation_covariance = [0 , 0 , 0, 0, 0, 0, 0, 0, 0]
            self.imuMsg.angular_velocity_covariance = [0, 0 , 0, 0 , 0, 0, 0 , 0 , 0]
            self.imuMsg.linear_acceleration_covariance = [0 , 0 , 0, 0 , 0, 0, 0 , 0 , 0]
            
            self.roll = 0
            self.pitch = 0
            self.yaw = self.th # şimdilik
            # Acceloremeter
            self.imuMsg.linear_acceleration.x = 0
            self.imuMsg.linear_acceleration.y = 0
            self.imuMsg.linear_acceleration.z = 0

            # Gyro
            self.imuMsg.angular_velocity.x = 0
            self.imuMsg.angular_velocity.y = 0
            self.imuMsg.angular_velocity.z = 0

            self.q = tf.transformations.quaternion_from_euler(self.roll,self.pitch,self.yaw)
            self.imuMsg.orientation.x = self.q[0] #magnetometer
            self.imuMsg.orientation.y = self.q[1]
            self.imuMsg.orientation.z = self.q[2]
            self.imuMsg.orientation.w = self.q[3]

            self.imuMsg.header.frame_id = "base_link"
            self.imuMsg.header.stamp = self.odom.header.stamp
            # Subscriber(s)
            rospy.Subscriber('/husky_velocity_controller/cmd_vel', Twist, self.callback)
            rospy.Subscriber('/odometry/gps', Odometry, self.gps_callback)
            # Publisher(s)
            self.odom_pub.publish(self.odom)
            self.odom_combined_pub.publish(self.pwcs)
            self.gps_pub.publish(self.gps_fix)
            self.waypoint_pub.publish(self.waypoint)  
            self.imu_pub.publish(self.imuMsg)         
            rate.sleep()


if __name__ == '__main__':
    VelocityController()