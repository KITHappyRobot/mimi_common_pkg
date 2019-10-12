#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: 人接近のタスク設計用ActionServerROSノード
#Author: Issei Iida
#Date: 2019/10/05
#Memo: 
#---------------------------------------------------------------------

#Python関連
import sys
#ROS関連
import rospy
import roslib
import smach
from smach import StateMachine
from smach_ros import ActionServerWrapper
from actionlib import *
from mimi_common_pkg.msg import *
from std_msgs.msg import String

#sys.path.append(roslib.packages.get_pkg_dir('mimi_common_pkg') + 'scripts')
sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import *


class FindPerson(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['find', 'not_find'],
                input_keys = ['goal_in'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state FIND_PERSON')
            if userdata.goal_in.data == 'start':
                speak('Finding person')
                rospy.sleep(2.0)
                speak('I found person')
                return 'find'
        except rospy.ROSInterruptException:
            pass

class GetCootdinate(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['get', 'not_get'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state GET_COORDINATE')
            return 'get'
        except rospy.ROSInterruptException:
            pass


class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['arrive', 'not_arrive'],
                input_keys = ['result_message'],
                output_keys = ['result_message'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state NAVIGATION')
            #パラメータ変更処理書く
            result = userdata.result_message
            result = 'success'
            userdata.result_message.data = result
            speak('I came close to person')
            return 'arrive'
        except rospy.ROSInterruptException:
            pass


def main():
    sm_top = StateMachine(
            outcomes = ['success',
                        'find_failed',
                        'navi_failed',
                        'get_failed',
                        'preempted'],
            input_keys = ['goal_message', 'result_message'],
            output_keys = ['result_message'])

    with sm_top:
        StateMachine.add(
                'FIND_PERSON',
                FindPerson(),
                transitions = {'find':'GET_COORDINATE',
                               'not_find':'find_failed'},
                remapping = {'goal_in':'goal_message'})

        StateMachine.add(
                'GET_COORDINATE',
                GetCootdinate(),
                transitions = {'get':'NAVIGATION',
                               'not_get':'get_failed'})

        StateMachine.add(
                'NAVIGATION',
                Navigation(),
                transitions = {'arrive':'success',
                               'not_arrive':'navi_failed'},
                remapping = {'result_message':'result_message'})

    asw = ActionServerWrapper(
            'approach_person',
            ApproachPersonAction,
            wrapped_container = sm_top,
            succeeded_outcomes = ['success'],
            aborted_outcomes = ['find_failed', 'get_failed', 'navi_failed'],
            preempted_outcomes = ['preempted'],
            goal_key = 'goal_message',
            result_key = 'result_message')

    asw.run_server()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        rospy.init_node('approach_person', anonymous = True)
        main()
    except rospy.ROSInterruptException:
        pass
