#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: 人接近のタスク設計用ActionServerROSノード
#Author: Issei Iida
#Date: 2019/10/13
#Memo: 台風19号ハギビス
#---------------------------------------------------------------------

#Python関連
import sys
import time
#ROS関連
import rospy
import rosparam
import roslib
import smach
import smach_ros
from smach import StateMachine
from smach_ros import ActionServerWrapper
from actionlib import *
from mimi_common_pkg.msg import *
from std_msgs.msg import String
#from get_distance_pcl.msg import Coordinate_xyz

sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import *
from common_action_client import *


class FindPerson(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['find', 'not_find'],
                input_keys = ['goal_in'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state FIND_PERSON')
            #result = findPersonAC()
            result = 'success'
            if result == 'success':
                #speak('I found person')
                rospy.loginfo('Find person')
                return 'find'
            else:
                #speak('I can`t find person')
                rospy.loginfo('Not find person')
                return 'not_find'
        except rospy.ROSInterruptException:
            pass

class GetCootdinate(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['get', 'not_get'],
                output_keys = ['coord_out'])
        #Subscriber
        #rospy.Subscriber('get_distance_pcl/Coordinate_xyz', Coordinate_xyz, self.personCoordCB)

        self.person_coord_x = 0.00
        self.person_coord_y = 0.00
        self.coord_list = []
        #Flag
        self.coordinate_flg = False

    def personCoordCB(self, receive_msg):
        self.person_coord_x = receive_msg.world_x
        self.person_coord_y = receive_msg.world_y
        self.coordinate_flg = True

    #def orientationCB(self, receive_msg):
    #    self.person_coord_z = receive_msg.pose.pose.orientation.z
    #    self.person_coord_w = receive_msg.pose.pose.orientation.w

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state GET_COORDINATE')
            timeout = time.time() + 10
            while not rospy.is_shutdown and self.coordinate_flg is False:
                rospy.loginfo('Waiting for coordinate')
                if time.time() > timeout:
                    rospy.loginfo('Time out')
                    #return 'not_get'
                    break
                rospy.sleep(1.0)
            self.coordinate_flg = False 
            self.coord_list.append(self.person_coord_x)
            self.coord_list.append(self.person_coord_y)
            userdata.coord_out = self.coord_list
            return 'get'
        except rospy.ROSInterruptException:
            pass


class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['arrive', 'not_arrive'],
                input_keys = ['result_message',
                              'coord_in'],
                output_keys = ['result_message'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state NAVIGATION')
            ap_result = userdata.result_message
            #パラメータ変更処理書く
            #rosparam.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', float(0.45))
            #result = navigationAC()
            result = 'success'
            if result == 'success':
                #speak('I came close to person')
                ap_result = result
                userdata.result_message.data = ap_result
                return 'arrive'
            elif result == 'failed':
                #speak('I can`t came close to person')
                ap_result = result
                userdata.result_message.data = ap_result
                return 'not_arrive'
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
                               'not_get':'get_failed'},
                remapping = {'coord_out':'person_coord'})

        StateMachine.add(
                'NAVIGATION',
                Navigation(),
                transitions = {'arrive':'success',
                               'not_arrive':'navi_failed'},
                remapping = {'result_message':'result_message',
                             'coord_in':'person_coord'})

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
