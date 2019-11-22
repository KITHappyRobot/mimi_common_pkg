#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: ActionClientを纏めたPythonスクリプト
# Author: Issei Iida
# Date: 2019/09/18
# Memo: 
#--------------------------------------------------------------------

#ROS関係
import rospy
import actionlib
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mimi_common_pkg.msg import *


def enterTheRoomAC(receive_msg):
    try:
        rospy.loginfo("Start EnterTheRoom")
        ac = actionlib.SimpleActionClient('enter_the_room', EnterTheRoomAction)
        ac.wait_for_server()

        goal = EnterTheRoomGoal()
        goal.distance = receive_msg

        ac.send_goal(goal)
        ac.wait_for_result()
    
        result = ac.get_result()
        if result.data == 'success':
            rospy.loginfo("Success EnterTheRoom")
            ac.cancel_goal()
            return 'success'
        else:
            rospy.loginfo("Failed EnterTheRoom")
            ac.cancel_goal()
            return 'failed'
    except rospy.ROSInterruptException:
        pass

def approachPersonAC():
    try:
        rospy.loginfo("Start ApproachPerson")
        ac = actionlib.SimpleActionClient('approach_person', ApproachPersonAction)
        ac.wait_for_server()

        goal = ApproachPersonGoal()
        goal.data = 'start'

        ac.send_goal(goal)
        ac.wait_for_result()

        result = ac.get_result()
        print result
        if result.data == 'success':
            rospy.loginfo("Success ApproachPerson")
            ac.cancel_goal()
            return 'success'
        elif result.data == 'aborted':
            rospy.loginfo("Aborted ApproachPerson")
            ac.cancel_goal()
            return 'aborted'
        else:
            return 'failed'
    except rospy.ROSInterruptException:
        pass


def findPersonAC():
    try: 
        rospy.loginfo('Start FindPerson')
        ac = actionlib.SimpleActionClient('find_person', FindPersonAction)
        ac.wait_for_server()

        goal = FindPersonGoal()
        goal.data = 'start'

        ac.send_goal(goal)
        ac.wait_for_result()

        result = ac.get_result()
        print result
        while not rospy.is_shutdown():
            if result.data == 'success':
                rospy.loginfo('Success FindPerson')
                ac.cancel_goal()
                return 'success'
            elif result.data == 'failed':
                rospy.loginfo('Failed FindPerson')
                ac.cancel_goal()
                return 'failed'
    except rospy.ROSInterruptException:
        pass


def navigationAC(coord_list):
    try:
        rospy.loginfo("Start Navigation")
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        ac.wait_for_server()
        #CostmapService
        clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        #Set Goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = coord_list[0]
        goal.target_pose.pose.position.y = coord_list[1]
        goal.target_pose.pose.orientation.z = coord_list[2]
        goal.target_pose.pose.orientation.w = coord_list[3]
        #Costmapを消去
        rospy.wait_for_service('move_base/clear_costmaps')
        clear_costmaps()
        rospy.sleep(1.0)
        #Goalを送信
        ac.send_goal(goal)
        state = ac.get_state()
        count = 0#<---clear_costmapsの実行回数をカウントするための変数
        while not rospy.is_shutdown():
            state = ac.get_state()
            if state == 1:
                rospy.loginfo('Got out of the obstacle')
                rospy.sleep(2.0)
            elif state == 3:
                rospy.loginfo('Navigation success!!')
                ac.cancel_goal()
                return 'success'
                state = 0
            elif state == 4:
                if count == 100:
                    count = 0
                    rospy.loginfo('Navigation Failed')
                    ac.cancel_goal()
                    return 'failed'
                else:
                    rospy.loginfo('Clear Costmaps')
                    clear_costmaps()
                    ac.send_goal(goal)
                    rospy.loginfo('Send Goal')
                    rospy.sleep(2.0)
                    count += 1
    except rospy.ROSInterruptException:
        pass
