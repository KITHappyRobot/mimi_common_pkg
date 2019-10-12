#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
#Title: ActionClientを纏めたPythonスクリプト
#Author: Issei Iida
#Date: 2019/09/18
#Memo: 
#--------------------------------------------------------------------

#ROS関係
import rospy
import roslib
import actionlib
from std_msgs.msg import String
from mimi_common_pkg.msg import *

def detectDoorOpenAC():
    try:
        rospy.loginfo("Start DetectDoorOpen")
        ac = actionlib.SimpleActionClient('detect_door_open', DetectDoorOpenAction)
        ac.wait_for_server()

        goal = DetectDoorOpenGoal()
        goal.data = 'start'

        ac.send_goal(goal)
        ac.wait_for_result()
    
        result = ac.get_result()
        if result.data == 'success':
            rospy.loginfo("Success DetectDoorOpen")
            return 'success'
        else:
            rospy.loginfo("Failed DetectDoorOpen")
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
            return 'success'
        elif result.data == 'aborted':
            rospy.loginfo("Aborted ApproachPerson")
            return 'aborted'
        else:
            return 'failed'
    except rospy.ROSInterruptException:
        pass

def findPerson():
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
        if result.data == 'success':
            rospy.loginfo('Success FindPerson')
            return 'success'
        else:
            rospy.loginfo('Failed FindPerson')
            return 'failed'
    except rospy.ROSInterruptException:
        pass
