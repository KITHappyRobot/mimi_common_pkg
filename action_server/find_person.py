#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: 人を探すためのActionServer
#Author: Issei Iida
#Date: 2019/10/11
#Memo: 人を見つけるまで回転する仕様
#---------------------------------------------------------------------

#Python関連ライブラリ
import sys
import time
#ROS関連ライブラリ
import rospy
import smach
from smach import StateMachine
from smach_ros import ActionServerWrapper
import actionlib
from std_msgs.msg import String
from mimi_common_pkg.msg import (FindPersonAction,
                                 FindPersonGoal,
                                 FindPersonFeedback,
                                 FindPersonResult) 

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import *


class Find(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['find',
                            'not_find'],
                input_keys = ['result_in'],
                output_keys = ['result_out'])
        #Subscriber
        rospy.Subscriber('/recog_obj', String, self.recogCB)
        #Flag
        self.person_flg = False
        self.kc = KobukiControl()

    def recogCB(self, receive_msg):
        obj_list = receive_msg.data.split(" ")
        for i in range(len(obj_list)):
            if obj_list[i] == 'person':
                self.person_flg = True

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state FIND')
            result = userdata.result_in
            m6Control(-0.2)
            timeout = time.time() + 30
            self.person_flg = False
            while not rospy.is_shutdown() and self.person_flg == False:
                self.kc.angularControl(0.3)
                rospy.loginfo('Finding...')
                rospy.sleep(0.1)
                #if time.time() > timeout:
                #    rospy.loginfo('**Time out!**')
                #    timeout = 0
                #    break
            self.kc.angularControl(0.0)
            m6Control(0.3)
            if self.person_flg == True:
                result.data = 'success'
                userdata.result_out = result
                return 'find'
            elif self.person_flg == False:
                result.data = 'failure'
                userdata.result_out.data = result
                return 'not_find'
        except rospy.ROSInterruptException:
            pass

def main():
    sm_top = StateMachine(
            outcomes = ['success',
                        'failure',
                        'preempted'],
            input_keys = ['goal_message',
                          'result_message'],
            output_keys = ['result_message'])

    with sm_top:
        StateMachine.add(
                'FIND',
                Find(),
                transitions = {'find':'success',
                               'not_find':'failure'},
                remapping = {'result_in':'result_message',
                             'result_out':'result_message'})

    asw = ActionServerWrapper(
            'find_person',
            FindPersonAction,
            wrapped_container = sm_top,
            succeeded_outcomes = ['success'],
            aborted_outcomes = ['failure'],
            preempted_outcomes = ['preempted'],
            goal_key = 'goal_message',
            result_key = 'result_message')

    asw.run_server()
    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('find_person', anonymous = True)
        main()
    except rospy.ROSInterruptExection:
        pass
