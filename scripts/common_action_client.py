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
            rospy.loginfo("Faild DetectDoorOpen")
            return 'faild'
    except rospy.ROSInterruptException:
        pass
