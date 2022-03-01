#! /usr/bin/env python3


import os
import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
IMAGE_WIDTH=1241
IMAGE_HEIGHT=376



import time
import cv2
import torch
from numpy import random
import torch.backends.cudnn as cudnn
import numpy as np
from models.experimental import attempt_load
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, plot_one_box, strip_optimizer, set_logging)
from utils.torch_utils import select_device, load_classifier, time_synchronized

from matplotlib import pyplot as plt

obj = 'bottle'
precision = 0.5

box_pub = None

def image_callback_1(image):
    global ros_image ,box_pub
    # rospy.loginfo("image.height %d, image.width %d",image.height, image.width)
    # rospy.loginfo("image len %d",len(image.data))
    ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    with torch.no_grad():
        results = model(ros_image)
        # print(results)
        for i, (im, pred) in enumerate(zip(results.imgs, results.pred)):
            s = f'image {i + 1}/{len(results.pred)}: {im.shape[0]}x{im.shape[1]} '  # string
            if pred.shape[0]:
                for *box, conf, cls in reversed(pred):  # xyxy, confidence, class
                    label = f'{results.names[int(cls)]} {conf:.2f}'
                    # print(box)
                    # print(label)
                    if(results.names[int(cls)] == obj and conf > precision):
                        # box_str = str(int(box[0].item()))+','+str(int(box[1].item()))+','+str(int(box[2].item()))+','+str(int(box[3].item()))
                        # print(box_str)
                        tem_data = Int16MultiArray()
                        tem_data.data = [int(box[0].item()),int(box[1].item()),int(box[2].item()),int(box[3].item())]
                        box_pub.publish(tem_data)
                        # print(box[0],box[1],box[2],box[3])
                        plot_one_box(box, ros_image, label=label, color=[0,255,0], line_thickness=3)
            else:
                s += '(no detections)'
        out_img = ros_image[:, :, [2, 1, 0]]
        cv2.imshow('YOLOV5', out_img)
        a = cv2.waitKey(1)

if __name__ == '__main__':
    device = select_device('cpu')
    torch.device('cpu')
    # model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
    model = torch.hub.load('./', 'custom', path='./yolov5s.pt', source='local')

    '''
    模型初始化
    '''
    rospy.init_node('ros_yolo')
    image_topic_1 = "/camera/color/image_raw"
    rospy.Subscriber(image_topic_1, Image, image_callback_1, queue_size=1, buff_size=52428800)
    box_pub = rospy.Publisher('/yolo_result_out', Int16MultiArray, queue_size=3)
    #rospy.init_node("yolo_result_out_node", anonymous=True)
    

    rospy.spin()
