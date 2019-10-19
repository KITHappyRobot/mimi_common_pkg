#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: 工大祭でのデモプレイ用ROSノード
#Author: Issei Iida
#Date: 2019/10/18
#Memo: 物体把持のデモプレイ+ クイズ付き
#---------------------------------------------------------------------

#Python
import sys
import codecs
#ROS
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool

sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/scripts/')
from common_function import *
from common_action_client import *


class Introduction(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_intro'])

    def execute(self, userdata):
        coord_list = searchLocationName('intro')
        navigationAC(coord_list)
        rospy.loginfo('Executing state INTRODUCTION')
        openjSpeak('こんにちは！生活支援ロボットの、ハッピー,ミミです！')
        openjSpeak('今から、物体把持のデモンストレーションを、行いたいと思います！')
        openjSpeak('暖かく、見守ってくださいね！')
        rospy.loginfo('Finish Introduction')
        return 'finish_intro'
        

class GraspDemo(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_grasp'])
        #Publisher
        self.pub_grasp_req = rospy.Publisher('/object/grasp_req', String, queue_size = 1)
        self.pub_change_pose = rospy.Publisher('/arm/changing_pose_req', String, queue_size = 1)
        #Subscriber
        rospy.Subscriber('/object/grasp_res', Bool, self.graspCB)
        rospy.Subscriber('/arm/changing_pose_res', Bool, self.changingPoseCB)

    def graspCB(self, receive_msg):
        self.grasp_flg = receive_msg.data

    def changingPoseCB(self, receive_msg):
        self.chang_pose_flg = receive_msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state GRASP_DEMO')
        coord_list = searchLocationName('chair')
        navigationAC(coord_list)
        self.pub_grasp_req.publish('jyagariko')
        while not rospy.is_shutdown() and self.grasp_flg == False:
            rospy.loginfo('Waiting topic...')
            rospy.sleep(2.0)
        self.grasp_flg = False
        rospy.loginfo('Finish grasp')
        rospy.sleep(1.0)
        coord_list = searchLocationName('person')
        navigationAC(coord_list)
        self.pub_change_pose.publish('give')
        while not rospy.is_shutdown() and self.chang_pose_flg == False:
            rospy.loginfo('Waiting topic...')
            rospy.sleep(2.0)
        self.grasp_flg = False
        openjSpea('どうぞ！')
        return 'finish_grasp'


class Quiz(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_quiz'])

    def execute(self, userdata):
        rospy.loginfo('Executing state QUIZ')
        return 'finish_quiz'


def main():
    sm_top = smach.StateMachine(
            outcomes = ['finish_demo'])
    with sm_top:
        smach.StateMachine.add(
                'INTRODUCTION',
                Introduction(),
                transitions = {'finish_intro':'GRASP_DEMO'})

        smach.StateMachine.add(
                'GRASP_DEMO',
                GraspDemo(),
                transitions = {'finish_grasp':'QUIZ'})

        smach.StateMachine.add(
                'QUIZ',
                Quiz(),
                transitions = {'finish_quiz':'finish_demo'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    try:
        rospy.init_node('demo', anonymous = True)
        main()
    except rospy.ROSInterruptException:
        pass
