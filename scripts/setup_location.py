#!/usr/bin/env python
# -*- coding: utf-8 -*-
#--------------------------------------------------------------------
#Title: Locationの名前・座標をパラメータに登録するROSノード
#Author: Issei Iida
#Date: 2019/09/05
#Memo:
#--------------------------------------------------------------------

#ROS関係
import rospy
import rosparam
import actionlib
import smach
import smach_ros
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['set_start'],
                             output_keys=['waiting_out_data'])
        #Subscriber
        rospy.Subscriber('/setup_location/location_name', String, self.getLocationNameCB)
        self.location_name = 'Null'

    def execute(self, userdata):
        rospy.loginfo("Executing state: WAITING")
        while not rospy.is_shutdown() and self.location_name == 'Null':
            rospy.loginfo("Waiting for topic...")
            rospy.sleep(1.5)
        userdata.waiting_out_data = self.location_name
        rospy.sleep(0.1)
        self.location_name = 'Null'
        return 'set_start'

    def getLocationNameCB(self, receive_msg):
        try:
            self.location_name = receive_msg.data
        except SyntaxError:
            pass


class Setting(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['set_complete', 'set_finish'],
                             input_keys=['setting_in_data'],
                             output_keys=['setting_out_data'])
        #Subscriber
        rospy.Subscriber('/odom', Odometry, self.getOdomCB)

        self.location_list = []
        self.name_list = []
        self.location_pose_x = 0.00
        self.location_pose_y = 0.00
        self.location_pose_z = 0.00
        self.location_pose_w = 0.00

    def execute(self, userdata):
        rospy.loginfo("Executing state: SETTING")
        while not rospy.is_shutdown():
            if userdata.setting_in_data == 'set_finish':
                rospy.loginfo("Finish Setting")
                userdata.setting_out_data = self.location_list
                print self.location_list
                return 'set_finish'
            else:
                if userdata.setting_in_data in self.name_list:
                    rospy.loginfo("LocationName already exists")
                    return 'set_complete'
                else:
                    rospy.loginfo("Add <" + userdata.setting_in_data + "> position")
                    second_list = []
                    second_list.append(userdata.setting_in_data)
                    self.name_list.append(userdata.setting_in_data)
                    #while not rospy.is_shutdown() and self.location_pose_x == 0.00:
                    #    rospy.loginfo("Waiting for Odometry...")
                    #    rospy.sleep(1.0)
                    #second_list.append(self.location_pose_x)
                    #second_list.append(self.location_pose_y)
                    #second_list.append(self.location_pose_z)
                    #second_list.append(self.location_pose_w)
                    second_list.append(1)
                    second_list.append(2)
                    second_list.append(3)
                    second_list.append(4)
                    self.location_list.append(second_list)
                    rospy.loginfo("<" + userdata.setting_in_data + "> " + str(second_list))
                    second_list = []
                    rospy.sleep(0.1)
                    return 'set_complete'

    def getOdomCB(self, receive_msg):
        try:
            self.location_pose_x = receive_msg.pose.pose.position.x
            self.location_pose_y = receive_msg.pose.pose.position.y
            self.location_pose_z = receive_msg.pose.pose.orientation.z
            self.location_pose_w = receive_msg.pose.pose.orientation.w
        except IndexError:
            pass


class Saving(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['save_finish'],
                             input_keys=['saving_in_data'])

    def execute(self, userdata):
        rospy.loginfo("Executing state: SAVING")
        while not rospy.is_shutdown():
            rospy.loginfo("Create ROSParameter")
            print userdata.saving_in_data
            rospy.set_param('/location_list', userdata.saving_in_data)
            #rosparam.dump_params('/home/athome/catkin_ws/src/common_pkg/config/common_function_params.yaml', '/location_list')
            rosparam.dump_params('/home/issei/catkin_ws/src/common_pkg/config/common_function_params.yaml', '/location_list')
            rospy.loginfo("Created!")
            return 'save_finish'


def main():
    rospy.init_node('setup_location', anonymous = True)

    sm = smach.StateMachine(outcomes=['finish_setup_location'])
    sm.userdata.location_name = 'Null'
    with sm:
        smach.StateMachine.add('WAITING',
                                Waiting(),
                                transitions={'set_start':'SETTING'},
                                remapping={'waiting_out_data':'location_name'})
        smach.StateMachine.add('SETTING',
                                Setting(),
                                transitions={'set_complete':'WAITING',
                                             'set_finish':'SAVING'},
                                remapping={'setting_in_data':'location_name',
                                            'setting_out_data':'location_list'})
        smach.StateMachine.add('SAVING',
                                Saving(),
                                transitions={'save_finish':'finish_setup_location'},
                                remapping={'saving_in_data':'location_list'})
    outcome = sm.execute()

if __name__ == '__main__':
    main()
