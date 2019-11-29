#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: 人がいる方向を向くためのActionServer
# Author: Issei Iida
# Date: 2019/11/21
# Memo:
#---------------------------------------------------------------------

#Pyhon関係
import sys
import time
#ROS関係
import rospy
from std_msgs.msg import String
from mimi_common_pkg.msg import LocalizeObjectAction, LocalizeObjectResult
from mimi_common_pkg.srv import RecognizeExistence
import actionlib

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import KobukiControl, m6Control


class LocalizeObjectAS():
    def __init__(self):
        #ActionServer
        self.sas = actionlib.SimpleActionServer(
                'localize_object',
                LocalizeObjectAction,
                execute_cb = self.execute,
                auto_start = False)
        #Subscriber
        self.sub_recog = rospy.Subscriber('/recog_obj', String, self.recogCB)
        #Service
        self.obj_recog = rospy.ServiceProxy('/object/recognize', RecognizeExistence)
        
        self.kc = KobukiControl()
        self.result = LocalizeObjectResult()
        self.data = RecognizeExistence()
        self.person_flg = False
        self.timeout = 0

        self.sas.start()

    def recogCB(self, receive_msg):
        obj_list = receive_msg.data.split(" ")
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True

    def detection(self, receive_msg):
        #self.person_flg = False
        self.timeout = time.time() + 30
        print receive_msg
        self.data.target = receive_msg
        result = self.obj_recog(self.data.target)
        while not rospy.is_shutdown() and result.existence == False:
            self.kc.angularControl(0.3)
            result = self.obj_recog(self.data.target)
            if time.time() > self.timeout:
                return 'failure'
        #self.person_flg = False
        return 'success'

    def execute(self, goal):
        try:
            print goal.data
            rospy.loginfo('Start LocalizeObject')
            m6Control(-0.2)
            rospy.loginfo('Start detection')
            result = self.detection(goal.data)
            self.kc.angularControl(0.0)
            rospy.loginfo('Finish detection')
            m6Control(0.3)
            if result is 'success':
                self.result.data = result
                self.sas.set_succeeded(self.result)
            else:
                result.data = result
                self.sas.set_succeeded(self.result)
            rospy.loginfo('Finish LocalizeObject')
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('localize_object', anonymous = True)
        fp_server = LocalizeObjectAS()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('**Interrupted**')
        pass
