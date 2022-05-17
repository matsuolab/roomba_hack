#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, LaserScan
from cv_bridge import CvBridge
from pytorchyolo import detect, models
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import cv2
import copy

def cb(data):
    print(data.header.frame_id)

def hoge():
    rospy.Subscriber('/scan', LaserScan, cb)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detection_mask', anonymous=True)
    hoge()