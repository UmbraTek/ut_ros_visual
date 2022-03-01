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
        arm = moveit_commander.MoveGroupCommander('manipulator')
        # get current pose     
        current_pose = arm.get_current_pose()
        # print(current_pose)
        rospy.loginfo(current_pose.pose)




if __name__ == "__main__":
    
    MoveItIkDemo()

    
    
