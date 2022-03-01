import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

intrinsics = rs2.intrinsics()
result = rs2.rs2_deproject_pixel_to_point(intrinsics, [pix[0], pix[1]], depth)
line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
print(intrinsics)