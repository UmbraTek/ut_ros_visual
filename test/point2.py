#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import ctypes
import struct
import rospy
import time

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    for field in data.fields:
        rospy.loginfo(field.name)
        rospy.loginfo(field.datatype)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z","rgb"), skip_nans=True)
    time.sleep(1)
    # print type(gen)
    image_data = []
    for p in gen:
        test = p[3]
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        pack = ctypes.c_uint32(i).value
        r = int((pack & 0x00FF0000)>> 16)
        g = int((pack & 0x0000FF00)>> 8)
        b = int((pack & 0x000000FF))
        image_data.append(r)
        image_data.append(g)
        image_data.append(b)
        print " x : %.3f  y: %.3f  z: %.3f r:%d g:%d b:%d" %(p[0],p[1],p[2],r,g,b)

def main():
    rospy.init_node('pcl_listener', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback_pointcloud)
    rospy.spin()

if __name__ == "__main__":
    main()
