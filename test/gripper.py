#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from utra_msg.srv import *


if __name__ == "__main__":
    rospy.wait_for_service('utra/gripper_state_set')
    try:
        gripper_state_set = rospy.ServiceProxy('utra/gripper_state_set', GripperStateSet)
        res = gripper_state_set(1)
        print(res)
        gripper_mv = rospy.ServiceProxy('utra/gripper_mv', Grippermv)
        res=gripper_mv(80)
        print(res)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)