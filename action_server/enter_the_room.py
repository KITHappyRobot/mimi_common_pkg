#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
#Title: ドアが開いたことを検出して入室するActionServer
#Author: Issei Iida
#Date: 2019/11/19
#Memo: ドアが閉まっていることを前提条件とする
#--------------------------------------------------------------------

#Python関係
import sys
#ROS関係
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from mimi_common_pkg.msg import (EnterTheRoomAction,
                                 EnterTheRoomResult)

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import KobukiControl, speak


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

        self.result = EnterTheRoomResult()
        self.front_laser_dist = 999.9

        self.sas.start()

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def detection(self, receive_msg):
        target_distance = self.front_laser_dist + receive_msg
        rospy.loginfo('Start detection')
        speak("Please open the door")
        while self.front_laser_dist <= target_distance:
            rospy.loginfo('Waiting for the door to open')
            rospy.sleep(1.0)
        rospy.loginfo('Door opened!')
        return target_distance

    def execute(self, goal):
        rospy.loginfo('Start EnterTheRoom')
        distance = self.detection(goal.data)
        speak("Thank you")
        kc.moveDistance(distance)
        rospy.loginfo('Entered the room')
        self.result.data = 'success'
        self.sas.set_succeeded(self.result)
        rospy.loginfo('Finish EnterTheRoom')

if __name__ == '__main__':
    try:
        rospy.init_node('enter_the_room', anonymous = True)
        ddo_server = EnterTheRoomAS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
