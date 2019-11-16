#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
#Title: 使用頻度が高い機能を纏めたPythonスクリプト
#Author: Issei Iida
#Date: 2019/09/07
#Memo: mimiの上(頭)から下(台車)の順に記述
#--------------------------------------------------------------------

#Python関係
import time
from yaml import load
import subprocess
import time
#ROS関係
import rospy
import rosparam
import actionlib
from std_srvs.srv import Empty
from std_msgs.msg import String, Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#Grobal
pub_speak = rospy.Publisher('/tts', String, queue_size = 1)
pub_m6 = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)


#話す *発話エンジンはpicottsで設定する
def speak(phrase):
    pub_speak.publish(phrase)
    rospy.sleep(2.0)


#m6(首のサーボモータ)の制御
def m6Control(value):
    #念のため型変換
    data = Float64()
    data = value
    rospy.sleep(0.1)
    pub_m6.publish(data)
    rospy.loginfo("Changing m6 pose")


class KobukiControl():
    def __init__(self):
        #Publisher
        self.pub_cmd_vel_mux = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)

        self.twist_value = Twist()

    def publishTwist(self, target, value):
        try:
            if target is 'linear':
                self.twist_value.linear.x = value
            elif target is 'angular':
                self.twist_value.angular.z = value
            rospy.sleep(0.1)
            self.pub_cmd_vel_mux.publish(self.twist_value)
        except rospy.ROSInterruptException:
            pass


#kobukiの回転
def angularControl(value):
    twist_cmd = Twist()
    twist_cmd.angular.z = value
    rospy.sleep(0.1)
    pub_cmd_vel_mux.publish(twist_cmd)


#文字列をパラメータの/location_dictから検索して位置座標を返す
def searchLocationName(target_name):
    rospy.loginfo("Search LocationName")
    #LocationListのyamlファイルを読み込む
    f = open('/home/issei/catkin_ws/src/mimi_common_pkg/config/location_dict.yaml')
    location_dict = load(f)
    f.close()
    if target_name in location_dict:
        print location_dict[target_name]
        rospy.loginfo("Retrun location_dict")
        return location_dict[target_name]
    else:
        rospy.loginfo("Not found <" + target_name + "> in LocationDict")
        return 'faild'
