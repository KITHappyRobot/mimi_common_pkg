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
from mimi_common_pkg.msg import FacePersonAction, FacePersonResult
import actionlib

sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import KobukiControl, m6Control




class FacePersonAS():
    def __init__(self):
        #ActionServer
        self.sas = actionlib.SimpleActionServer(
                'face_person',
                FacePersonAction,
                execute_cb = self.execute,
                auto_start = False)
        #Subscriber
        self.sub_recog = rospy.Subscriber('/recog_obj', String, self.recogCB)
        
        self.kc = KobukiControl()
        self.result = FacePersonResult()
        self.person_flg = False
        self.timeout = 0

    def recogCB(self, receive_msg):
        obj_list = receive_msg.data.split(" ")
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True

    def detection(self):
        self.person_flg = False
        self.timeout = time.time() + 30
        while not rospy.is_shutdown() and self.person_flg == False:
            self.kc.angularControl(0.3)
            if time.time() > self.timeout:
                return 'failure'
        self.person_flg = False
        return 'success'

    def execute(self, userdata):
        try:
            rospy.loginfo('Start FacePerson')
            m6Control(-0.2)
            rospy.loginfo('Start detection')
            result = detection()
            self.kc.angularControl(0.0)
            rospy.loginfo('Finish detection')
            m6Control(0.3)
            if result is 'success':
                self.result.data = result
                self.sas.set_succeeded(self.result)
            else:
                result.data = result
                self.sas.set_succeeded(self.result)
            rospy.loginfo('Finish FacePerson')
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('face_person', anonymous = True)
        fp_server = FacePersonAS()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('**Interrupted**')
        pass
