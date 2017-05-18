#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf
import random



def main():

    val  = random.random()
    # val = 0

    rospy.init_node("imu_node")
    pub = rospy.Publisher('imu/data', Imu, queue_size = 10)

    imuMsg = Imu()
    imuMsg.orientation_covariance = [0 , 0 , 0, 0, 0, 0, 0, 0, 0]
    imuMsg.angular_velocity_covariance = [0, 0 , 0, 0 , 0, 0, 0 , 0 , 0]
    imuMsg.linear_acceleration_covariance = [0 , 0 , 0, 0 , 0, 0, 0 , 0 , 0]

    print("Sleeping 1 second")
    rospy.sleep(1)
    # while(1)
    rate  = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Will be obtained from sensor
        roll = val
        pitch = val
        yaw = val
        # Acceloremeter
        imuMsg.linear_acceleration.x = val
        imuMsg.linear_acceleration.y = val
        imuMsg.linear_acceleration.z = val

        # Gyro
        imuMsg.angular_velocity.x = val
        imuMsg.angular_velocity.y = val
        imuMsg.angular_velocity.z = val

        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        imuMsg.orientation.x = q[0] #magnetometer
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'base_link'
        rospy.loginfo(imuMsg)
        pub.publish(imuMsg)
        rate.sleep()



if __name__ == '__main__':
    main()