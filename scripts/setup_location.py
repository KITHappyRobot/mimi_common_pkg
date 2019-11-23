#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
# Title: Locationの名前・座標をパラメータに登録するROSノード
# Author: Issei Iida
# Date: 2019/11/04
# Memo: location_dictはキーが場所名、値が座標（[x,y,z,w]）
#       Smachを用いたプログラムの処女作
#--------------------------------------------------------------------

#ROS関係
import rospy
import rosparam
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import smach_ros
import smach


class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['set_start'],
                output_keys = ['waiting_out_data'])
        #Subscriber
        rospy.Subscriber('/setup_location/location_name', String, self.getLocationNameCB)
        self.location_name = 'null'

    def getLocationNameCB(self, receive_msg):
        self.location_name = receive_msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: WAITING')
        self.location_name = 'null'
        while not rospy.is_shutdown() and self.location_name == 'null':
            rospy.loginfo('Waiting for topic...')
            rospy.sleep(1.0)
        userdata.waiting_out_data = self.location_name
        return 'set_start'


class Setting(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['set_complete', 'set_finish'],
                input_keys = ['setting_in_data'],
                output_keys = ['setting_out_data'])
        #Subscriber
        rospy.Subscriber('/odom', Odometry, self.getOdomCB)

        self.location_dict = {}
        self.location_pose_x = 0.00
        self.location_pose_y = 0.00
        self.location_pose_z = 0.00
        self.location_pose_w = 0.00

    def getOdomCB(self, receive_msg):
        if receive_msg.child_frame_id == 'base_footprint':
            self.location_pose_x = receive_msg.pose.pose.position.x
            self.location_pose_y = receive_msg.pose.pose.position.y
            self.location_pose_z = receive_msg.pose.pose.orientation.z
            self.location_pose_w = receive_msg.pose.pose.orientation.w

    def getMapCoordinateCB(self, receive_msg):
        self.location_pose_x = receive_msg.x
        self.location_pose_y = receive_msg.y

    def execute(self, userdata):
        rospy.loginfo('Executing state: SETTING')
        while not rospy.is_shutdown():
            if userdata.setting_in_data == 'set_finish':
                rospy.loginfo('Finish Setting')
                userdata.setting_out_data = self.location_dict
                return 'set_finish'
            elif userdata.setting_in_data in self.location_dict:
                rospy.loginfo('LocationName already exists')
                return 'set_complete'
            else:
                rospy.loginfo('Add <' + userdata.setting_in_data + '> position')
                while not rospy.is_shutdown() and self.location_pose_x == 0.00:
                    rospy.loginfo('Waiting for Odometry...')
                    rospy.sleep(1.0)
                self.location_dict[userdata.setting_in_data] = []
                self.location_dict[userdata.setting_in_data].append(self.location_pose_x)
                self.location_dict[userdata.setting_in_data].append(self.location_pose_y)
                self.location_dict[userdata.setting_in_data].append(self.location_pose_z)
                self.location_dict[userdata.setting_in_data].append(self.location_pose_w)
                print self.location_dict[userdata.setting_in_data]
                return 'set_complete'


class Saving(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['save_finish'],
                input_keys = ['saving_in_data'])

    def execute(self, userdata):
        try:
            rospy.loginfo('Executing state: SAVING')
            rospy.set_param('/location_dict', userdata.saving_in_data)
            rosparam.dump_params('/home/athome/catkin_ws/src/mimi_common_pkg/config/location_dict.yaml', '/location_dict')
            rospy.loginfo('Saving complete')
            return 'save_finish'
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass


def main():
    sm = smach.StateMachine(outcomes = ['finish_setup_location'])
    sm.userdata.location_name = 'null'
    with sm:
        smach.StateMachine.add(
                'WAITING',
                Waiting(),
                transitions = {'set_start':'SETTING'},
                remapping = {'waiting_out_data':'location_name'})
        
        smach.StateMachine.add(
                'SETTING',
                Setting(),
                transitions = {'set_complete':'WAITING',
                               'set_finish':'SAVING'},
                remapping = {'setting_in_data':'location_name',
                             'setting_out_data':'location_list'})
        
        smach.StateMachine.add(
                'SAVING',
                Saving(),
                transitions = {'save_finish':'finish_setup_location'},
                remapping = {'saving_in_data':'location_list'})
    
    outcome = sm.execute()

if __name__ == '__main__':
    try:
        rospy.init_node('setup_location', anonymous = True)
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('**Interrupted**')
        pass
