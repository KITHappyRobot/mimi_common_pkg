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
from mimi_common_pkg.msg import LocalizePersonAction, PersonLocalizeResult
from object_recognizer.srv import RecognizeExistence
import actionlib

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import KobukiControl, m6Control


class LocalizePersonAS():
    def __init__(self):
        #ActionServer
        self.sas = actionlib.SimpleActionServer(
                'localize_person',
                LocalizePersonAction,
                execute_cb = self.execute,
                auto_start = False)
        #Subscriber
        self.sub_recog = rospy.Subscriber('/recog_obj', String, self.recogCB)
        #Service
        self.obj_recog = rospy.ServiceProxy('/object_recognize', RecognizeExistence)
        
        self.kc = KobukiControl()
        self.result = LocalizePersonResult()
        self.data = RecognizeExistence()
        self.person_flg = False
        self.timeout = 0

        self.sas.start()

    def recogCB(self, receive_msg):
        obj_list = receive_msg.data.split(" ")
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True

    def detection(self):
        #self.person_flg = False
        self.timeout = time.time() + 30
        self.data.target = 'person'
        result = self.obj_recog(self.data)
        while not rospy.is_shutdown() and result.existence == False:
            self.kc.angularControl(0.3)
            result = self.obj_recog(self.data)
            if time.time() > self.timeout:
                return 'failure'
        #self.person_flg = False
        return 'success'

    def execute(self, goal):
        try:
            rospy.loginfo('Start LocalizePerson')
            m6Control(-0.2)
            rospy.loginfo('Start detection')
            result = self.detection()
            self.kc.angularControl(0.0)
            rospy.loginfo('Finish detection')
            m6Control(0.3)
            if result is 'success':
                self.result.data = result
                self.sas.set_succeeded(self.result)
            else:
                result.data = result
                self.sas.set_succeeded(self.result)
            rospy.loginfo('Finish LocalizePerson')
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('localize_person', anonymous = True)
        fp_server = LocalizePersonAS()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('**Interrupted**')
        pass
