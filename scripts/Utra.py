#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 The UmbraTek Inc. All Rights Reserved.
#
# Software License Agreement (BSD License)
#
# Author: johnson huang<johnson@umbratek.com>
# =============================================================================

import rospy, sys
import moveit_commander
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import PoseStamped, Pose
from utra_msg.srv import *
import tf
DEBUG = False
base = '/base_link'
target = '/bottle'
#api reference https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py

class Utra:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
 
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        
        # set tolerance
        self.arm.set_goal_joint_tolerance(0.001)

        # set max velocity acceleration
        self.arm.set_max_acceleration_scaling_factor(0.1)
        self.arm.set_max_velocity_scaling_factor(0.1)

        self.step = 0

        self.init_tf()

        try:
            self.gripper_state_set = rospy.ServiceProxy('utra/gripper_state_set', GripperStateSet)
            res = self.gripper_state_set(1)
            print(res)
            self.gripper_mv = rospy.ServiceProxy('utra/gripper_mv', Grippermv)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    def init_tf(self):
        self.listener = tf.TransformListener()
        
    
    def get_bottle_position(self):
        try:
            self.listener.waitForTransform(base,target,rospy.Time(0),rospy.Duration(5))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("waitForTransform ERROR")
            return False
        try:
            (trans,rot) = self.listener.lookupTransform(base, target, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("lookupTransform ERROR")
            return False
        print(trans)
        return trans

    def plan_start_pose(self):
        self.step = 1
        return self.step,'Go to Start pose'

    def next_pose(self):
        self.step = self.step +1
        if(self.step == 1):
            return self.step,'Go to Start pose'
        if(self.step == 2):
            return self.step,'Graspe bottle'
        if(self.step == 3):
            return self.step,'Graspe up'
        if(self.step == 4):
            return self.step,'Get out'
        if(self.step == 5):
            self.step = 4
            return self.step,'Get out'
        return self.step,''
    def plan(self):
        if(self.step == 1):
            #start pose
            joint_positions = [-1.57, 0, 1.57, 0, 1.57, 0]
            self.arm.set_joint_value_target(joint_positions)
            self.plan_msg = self.arm.plan()
            return True,'ok'
        if(self.step == 2):
            # go to graspe bottle
            return self._goto_object()
        if(self.step == 3):
            joint_positions = [-1.57, 0, 1.57, 0, 1.57, 0]
            self.arm.set_joint_value_target(joint_positions)
            self.plan_msg = self.arm.plan()
            return True,'ok'
        if(self.step == 4):
            joint_positions = [0, 0, 1.57, 0, 1.57, 0]
            self.arm.set_joint_value_target(joint_positions)
            self.plan_msg = self.arm.plan()
            return True,'ok'
            
    
    def _goto_object(self):
        # move to object 
        #-------------------------------------------------------------------
        end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo(end_effector_link)
        # set pose reference Coordinate System
        reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(reference_frame)
        
        self.arm.allow_replanning(True)
        
        # set tolerance
        self.arm.set_goal_position_tolerance(0.001) # meter unit
        self.arm.set_goal_orientation_tolerance(0.01) # rad unit
        

        # get current pose     
        current_pose = self.arm.get_current_pose()
        # print(current_pose)
        rospy.loginfo(current_pose.pose.position)
        # set target pose
        trans = self.get_bottle_position()
        if(trans == False):
            return False,'fail to get target position'
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()    
        target_pose.pose.position.x = trans[0]
        target_pose.pose.position.y = trans[1]
        target_pose.pose.position.z = trans[2] - 0.05
        target_pose.pose.orientation.x = current_pose.pose.orientation.x
        target_pose.pose.orientation.y = current_pose.pose.orientation.y
        target_pose.pose.orientation.z = current_pose.pose.orientation.z
        target_pose.pose.orientation.w = current_pose.pose.orientation.w
        
        
        self.arm.set_start_state_to_current_state()
        
        # set to target pose and move
        self.arm.set_pose_target(target_pose,end_effector_link)
        self.plan_msg = self.arm.plan()
        return True,'ok'
        

    def execute_plan(self):
        if(self.step == 1):
            self.gripper_mv(80)
        if(self.step == 3):
            self.gripper_mv(55)
            rospy.sleep(2)
        ret = self.arm.execute(self.plan_msg)
        if(self.step == 4):
            self.gripper_mv(80)
        return ret

    def close(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    
