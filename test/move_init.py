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
#api reference https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py

class MoveItFkDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('utra_moveit_fk_demo', anonymous=True)
 
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        
        # set tolerance
        self.arm.set_goal_joint_tolerance(0.001)

        # set max velocity acceleration
        self.arm.set_max_acceleration_scaling_factor(0.1)
        self.arm.set_max_velocity_scaling_factor(0.1)

        # go to home pose
        self.arm.set_named_target('home')
        self.arm.go()

        # go to this joints, plan first, and excute
        joint_positions = [-1.43, 0.20, 1.90, 1.06, 1.61, -0.0]
        self.arm.set_joint_value_target(joint_positions)
        plan_msg = self.arm.plan()
        rospy.sleep(2)
        self.arm.execute(plan_msg)

        # go to home pose, go immediately
        # joint_positions = [0, 0, 0, 0, 0, 0]
        # self.arm.set_joint_value_target(joint_positions)
        # self.arm.go()


        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    

if __name__ == "__main__":
    try:
        fk_demo = MoveItFkDemo()
       
    except rospy.ROSInterruptException:
        pass
