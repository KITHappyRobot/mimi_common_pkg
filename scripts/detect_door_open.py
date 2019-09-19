#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
#Title: ドアが開いたことを検出するActionServer
#Author: Issei Iida
#Date: 2019/09/10
#Memo: front_laser_distが最初のドアとの距離＋50cmになった時、
#  　  ドアが開いたと判断する仕様
#--------------------------------------------------------------------

#Python関係
import sys
#ROS関係
import rospy
import roslib
import actionlib
from std_msgs.msg import String
from mimi_common_pkg.msg import DetectDoorOpenAction, DetectDoorOpenResult, DetectDoorOpenFeedback 

sys.path.append(roslib.packages.get_pkg_dir('mimi_common_pkg') + 'scripts')
from common_function import *


class DetectDoorOpenAS():
    def __init__(self):
        #Subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserCB)
        #Make ActionServer
        self.sas = actionlib.SimpleActionServer('detect_door_open',
                DetectDoorOpenAction,
                execute_cb = self.execute,
                auto_start = False)

        self.feedback = DetectDoorOpenFeedback()
        self.result = DetectDoorOpenResult()
        self.front_laser_dist = 999.9

        self.sas.start()

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self, goal):
        print goal.data
        #while not rospy.is_shutdown() and self.front_laser_dist == 999.9:
        #    self.feedback.data = 'waiting_front_laser'
        #    self.sas.publish_feedback(self.feedback)
        #initial_distance = self.front_laser_dist
        speak("Please open the door")
        #while not rospy.is_shutdown() and self.front_laser_dist <= initial_distance + 0.88:#試走場のドアの幅を参考
        #    self.feedback.data = 'waiting_door_open'
        #    self.sas.publish_feedback(self.feedback)
        rospy.sleep(2.0)
        speak("Thank you")
        rospy.sleep(0.1)
        self.result.data = 'success'
        self.sas.set_succeeded(self.result)


if __name__ == '__main__':
    try:
        rospy.init_node('detect_door_open', anonymous = True)
        ddo_server = DetectDoorOpenAS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
