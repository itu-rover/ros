#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import asyncore
import roverlib
from roverlib.server import EchoServer
from roverlib.serial_com import SerialNode
from roverlib.settings import *
from std_msgs.msg import String


class RobotController(object):
    def __init__(self):
        rospy.init_node('robot_controller')
        self.pub = rospy.Publisher('from_robot_controller', String, queue_size=10)
        self.recevied_data = None
        self.run_serial()
        self.controller()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        print(rospy.get_caller_id() + "   I heard %s", data.data)
        self.received_data = data.data

    def controller(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            try:
                server_data = self.serial_node.server.handler.data
            except Exception as e:
                print(e)
                server_data = None
            rospy.Subscriber("from_motor_controller", String, self.callback)
            data = "From TCP: %s" % str(server_data)
            #rospy.loginfo(data)
            self.pub.publish(data)
            rate.sleep()

    def run_serial(self):
        tcp_server = EchoServer(HOST, PORT)
        self.serial_node = SerialNode(tcp_server)
        communication_thread = threading.Thread(target=self.serial_node.run)
        communication_thread.daemon = True
        communication_thread.start()


if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
