#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ドアが開いたことを検出して入室するActionServer
# Author: Issei Iida
# Date: 2019/11/19
# Memo: ドアが閉まっていることを前提条件とする
#--------------------------------------------------------------------

#Python関係
import sys
#ROS関係
import rospy
import actionlib
from mimi_common_pkg.msg import EnterTheRoomAction, EnterTheRoomResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import *


class EnterTheRoomAS():
    def __init__(self):
        #Subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laserCB)
        #ActionServer
        self.sas = actionlib.SimpleActionServer(
                'enter_the_room',
                EnterTheRoomAction,
                execute_cb = self.execute,
                auto_start = False)

        self.kc = KobukiControl()
        self.result = EnterTheRoomResult()
        self.front_laser_dist = 999.9

        self.sas.start()

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def detection(self, receive_msg):
        #-0.05は固定データ
        target_distance = self.front_laser_dist + receive_msg - 0.05
        speak("Please open the door")
        rospy.loginfo('Start detection')
        while self.front_laser_dist <= target_distance:
            rospy.loginfo('Waiting for the door to open')
            rospy.sleep(1.0)
        rospy.loginfo('Door opened')
        return target_distance

    def execute(self, goal):
        try:
            rospy.loginfo('Start EnterTheRoom')
            distance = self.detection(goal.distance)
            speak('Thank you')
            self.kc.moveDistance(distance)
            self.result.data = 'success'
            self.sas.set_succeeded(self.result)
            rospy.loginfo('Finish EnterTheRoom')
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('enter_the_room', anonymous = True)
        ddo_server = EnterTheRoomAS()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('**Interrupted**')
        pass
