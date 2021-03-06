#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String


class MotorController(object):
    def __init__(self):
        rospy.init_node('motor_controller')
        self.pub = rospy.Publisher('from_motor_controller', String, queue_size=10)
        self.sending_data = None
        self.controller()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        print(rospy.get_caller_id() + "  I heard %s", data.data)
        self.sending_data = "I am listener! %s" % data.data
    
    def controller(self):
        rate = rospy.Rate(10) # 10hz
    
        while not rospy.is_shutdown():
            rospy.Subscriber("from_robot_controller", String, self.callback)
            rospy.loginfo(self.sending_data)
            self.pub.publish(self.sending_data)
            rate.sleep()


if __name__ == '__main__':
    MotorController()
