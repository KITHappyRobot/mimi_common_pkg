#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: 人接近のタスク設計用ActionServerROSノード
#Author: Issei Iida
#Date: 2019/10/05
#Memo: 
#---------------------------------------------------------------------

#Python関係
import sys
#ROS関係ライブラリ
import rospy
import roslib
import actionlib
from std_msgs.msg import String
from mimi_common_pkg.msg import

sys.path.append(roslib.packages.get_pkg_dir('mimi_common_pkg') + 'scripts')
from common_function import *



def main():


if __name__ == '__main__':
    try:
        rospy.init_node('approach_person', anonymous = True)
        main()
    except rospy.ROSInterruptException:
        pass
