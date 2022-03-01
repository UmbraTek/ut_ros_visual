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
import tf
#api reference https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py


class MoveItIkDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('utra_moveit_ik_demo')
        self.getObjectPosition()
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

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()    
        target_pose.pose.position.x = -0.507355808324
        target_pose.pose.position.y = -0.402044534949
        target_pose.pose.position.z = 0.366763986811
        target_pose.pose.orientation.x = 0.856486638918
        target_pose.pose.orientation.y = -0.337532323556
        target_pose.pose.orientation.z = -0.142070574405
        target_pose.pose.orientation.w = 0.363756126819
        
      
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

    def getObjectPosition(self):
        base = '/base_link'
        target = '/bottle'
        # listener = tf.TransformListener()
        # try:
        #     listener.waitForTransform(base,target,rospy.Time(0),rospy.Duration(50))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.logerr("waitForTransform ERROR")
        # try:
        #     (trans,rot) = listener.lookupTransform(base, target, rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.logerr("lookupTransform ERROR")
        # self.target_pose = trans
        # rospy.loginfo(trans)
        self.target_pose = [0.0393187071289324, -0.6765794818772575, 0.1679779037433815]

if __name__ == "__main__":
    
    MoveItIkDemo()

    
    
