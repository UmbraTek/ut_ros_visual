#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import tf
import numpy as np
import time
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

target = (0,0,0)
refresh_time = 0

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic,br):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.intrinsics = None
        self.pix = None
        self.last_time_get_pix = time.time()
        self.pix_grade = None
        rospy.Subscriber('/yolo_result_out', Int16MultiArray, self.yolo_result_out_cb)
        self.br = br

    def imageDepthCallback(self, data):
        global refresh_time,target
        try:
            # rospy.loginfo('width: %d,height: %d, data len : %d ',data.width,data.height,len(data.data))
            # rospy.loginfo(data.encoding)
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # rospy.loginfo(cv_image.shape)
            now_time = time.time()
            diff_time = now_time - self.last_time_get_pix
            # rospy.loginfo(diff_time)
            if(self.pix and diff_time<1):
                pix = self.pix
                # pix = (418,336)
                line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                

                if self.intrinsics:
                    depth = cv_image[pix[1], pix[0]]
                    result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                    line += '  Coordinate: %.2f %.2f %.2f.' % (result[0], result[1], result[2])
                    if(depth!=0):
                        target = (result[0]/1000, result[1]/1000, result[2]/1000)
                        refresh_time = time.time()
                        # self.br.sendTransform((result[0]/1000, result[1]/1000, result[2]/1000),
                        #     (0.0, 0.0, 0.0, 1.0),
                        #     rospy.Time.now(),
                        #     "/bottle",
                        #     "/camera_color_optical_frame")
                        # rospy.loginfo(line)
                if (not self.pix_grade is None):
                    line += ' Grade: %2d' % self.pix_grade
                line += '\n'
                # rospy.loginfo(line)
                # sys.stdout.write(line)
                # sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return



    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def yolo_result_out_cb(self,result):
        # rospy.loginfo(result)
        intarr = result.data
        x = int((intarr[2]-intarr[0])/2 + intarr[0])
        y = int((intarr[3]-intarr[1])/2 + intarr[1])
        self.pix = (x,y)
        self.last_time_get_pix = time.time()
        # rospy.loginfo(self.pix)

def main(br):
    global refresh_time,target 
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    
    listener = ImageListener(depth_image_topic, depth_info_topic,br)

    # rospy.spin()
    rate = rospy.Rate(10.0)
    pre_refresh_time = 0
    count = 0
    while not rospy.is_shutdown():
        if(pre_refresh_time != refresh_time):
            pre_refresh_time = refresh_time
            count = 0
            br.sendTransform(target,
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/bottle",
                            "/camera_color_optical_frame")
        else:
            count = count + 1
            if(count>30):
                pass
            else:
                br.sendTransform(target,
                                    (0.0, 0.0, 0.0, 1.0),
                                    rospy.Time.now(),
                                    "/bottle",
                                    "/camera_color_optical_frame")
        rate.sleep()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    # transformlistener = tf.TransformListener()
    # try:
    #     transformlistener.waitForTransform('/base_link','/camera_color_frame',rospy.Time(0),rospy.Duration(50))
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     rospy.logerr("waitForTransform ERROR")
    # try:
    #     (trans,rot) = transformlistener.lookupTransform('/base_link', '/camera_color_frame', rospy.Time(0))
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     rospy.logerr("lookupTransform ERROR")
    # rospy.loginfo(trans)
    # rospy.loginfo(rot)
    br = tf.TransformBroadcaster()
    main(br)
