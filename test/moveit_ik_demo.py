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
from geometry_msgs.msg import PoseStamped, Pose
#api reference https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py


class MoveItIkDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('utra_moveit_ik_demo')
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        end_effector_link = arm.get_end_effector_link()
        rospy.loginfo(end_effector_link)
        # set pose reference Coordinate System
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
        arm.allow_replanning(True)
        
        # set tolerance
        arm.set_goal_position_tolerance(0.001) # meter unit
        arm.set_goal_orientation_tolerance(0.01) # rad unit
       
        # set max velocity acceleration
        arm.set_max_acceleration_scaling_factor(0.1)
        arm.set_max_velocity_scaling_factor(0.1)

        # go to the init pose
        arm.set_named_target('init')
        arm.go()
        rospy.sleep(3)

        # get current pose     
        current_pose = arm.get_current_pose()
        # print(current_pose)
        
        # set target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()    
        target_pose.pose.position.x = current_pose.pose.position.x
        target_pose.pose.position.y = current_pose.pose.position.y
        target_pose.pose.position.z = current_pose.pose.position.z+0.1 # move down 0.1m
        target_pose.pose.orientation.x = current_pose.pose.orientation.x
        target_pose.pose.orientation.y = current_pose.pose.orientation.y
        target_pose.pose.orientation.z = current_pose.pose.orientation.z
        target_pose.pose.orientation.w = current_pose.pose.orientation.w
        
      
        arm.set_start_state_to_current_state()
        
        # set to target pose and move
        arm.set_pose_target(target_pose,end_effector_link)
        traj = arm.plan()
        can_excute = raw_input("can excute , input y (yes) , n (no)\n")
        rospy.loginfo(can_excute)
        if(can_excute == 'y'):
            arm.execute(traj)
            rospy.sleep(1)
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
