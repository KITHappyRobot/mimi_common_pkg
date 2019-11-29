#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: 使用頻度が高い処理を纏めたPythonスクリプト
# Author: Issei Iida
# Date: 2019/09/07
# Memo: mimiの上(頭)から下(台車)の順に記述
#--------------------------------------------------------------------

#Python関係
import time
from yaml import load
#ROS関係
import rospy
import rosparam
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


#Grobal
pub_speak = rospy.Publisher('/tts', String, queue_size = 1)
pub_m6 = rospy.Publisher('/m6_controller/command', Float64, queue_size = 1)


#話す *発話エンジンはpicottsで設定する
def speak(phrase):
    pub_speak.publish(phrase)
    rospy.sleep(2.0)


class ActionPlan():
    def __init__(self):
        self.listen_count = 1
        self.listen_result = 'ERROR'

    def fixSentence(self):
        self.listen_result = gd.FixSentence()
        print self.listen_result
        return self.listen_result

    def execute(self):
        rospy.loginfo('Start ActionPlan')
        #聞き取りは３回行う
        while not rospy.is_shutdown() and self.listen_count <= 3:
            result = self.fixSentence()
            if self.listen_result == 'ERROR':
                rospy.loginfo('Listening Failed')
                speak('One more time please')
                self.listen_count += 1
            else:
                rospy.loginfo('Listening Success')
                return self.listen_result
        return 'failure'


#m6(首のサーボモータ)の制御
def m6Control(value):
    data = Float64()
    data = value
    rospy.sleep(0.1)
    pub_m6.publish(data)


#kobukiの制御
class KobukiControl():
    def __init__(self):
        #Publisher
        self.pub_cmd_vel_mux = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)

        self.twist_value = Twist()

    #ただ前進
    def linearControl(self, value):
        self.twist_value.linear.x = value
        rospy.sleep(0.1)
        self.pub_cmd_vel_mux.publish(self.twist_value)

    #ただ回転
    def angularControl(self, value):
        self.twist_value.angular.z = value
        rospy.sleep(0.1)
        self.pub_cmd_vel_mux.publish(self.twist_value)

    #指定した距離だけ前後移動
    def moveDistance(self, distance):
        try:
            #absは絶対値求める関数
            target_time = abs(distance / 0.2)
            if distance >0:
                self.twist_value.linear.x = 0.24
            elif distance < 0:
                self.twist_value.linear.x = -0.24
            rate = rospy.Rate(30)
            start_time = time.time()
            end_time = time.time()
            while end_time - start_time <= target_time:
                self.pub_cmd_vel_mux.publish(self.twist_value)
                end_time = time.time()
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


#文字列をパラメータの/location_dictから検索して位置座標を返す
def searchLocationName(target_name):
    rospy.loginfo("Search LocationName")
    #location_dictのyamlファイルを読み込む
    f = open('/home/athome/catkin_ws/src/mimi_common_pkg/config/location_dict.yaml')
    location_dict = load(f)
    f.close()
    if target_name in location_dict:
        print location_dict[target_name]
        rospy.loginfo("Retrun location_dict")
        return location_dict[target_name]
    else:
        rospy.loginfo("Not found <" + target_name + "> in location_dict")
        return 'faild'
