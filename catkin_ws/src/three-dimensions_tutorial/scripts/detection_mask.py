#!/usr/bin/env python3
import copy
from typing import List

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from ultralytics.engine.results import Results


class DetectionMask:
    def __init__(self):
        rospy.init_node('detection_mask', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)
        self.masked_depth_pub = rospy.Publisher('/masked_depth/image', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/masked_depth/camera_info', CameraInfo, queue_size=10)

        # Subscriber
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        cam_info = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
        message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, cam_info], 10, 1.0).registerCallback(self.callback_rgbd)

        self.bridge = CvBridge()
        self.rgb_image, self.depth_image, self.camera_info = None, None, None

        self.model = YOLO('yolov8n.pt')

    def callback_rgbd(self, data1, data2, data3):
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array

        self.camera_info = data3

    def process(self):
        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue

            tmp_rgb_image = copy.copy(self.rgb_image)
            tmp_depth_image = copy.copy(self.depth_image)
            tmp_camera_info = copy.copy(self.camera_info)

            depth_mask = np.zeros_like(tmp_depth_image)

            results: List[Results] = self.model.predict(self.rgb_image)

            # plot bounding box
            tmp_image = copy.deepcopy(self.rgb_image)
            for result in results:
                boxes = result.boxes.cpu().numpy()
                names = result.names
                if len(boxes.xyxy) == 0:
                    continue
                x1, y1, x2, y2 = map(int, boxes.xyxy[0][:4])
                cls_pred = boxes.cls[0]
                tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                tmp_image = cv2.putText(tmp_image, names[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                depth_mask[y1:y2, x1:x2] = 1

            # publish image
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            self.detection_result_pub.publish(detection_result)

            masked_depth = np.where(depth_mask, tmp_depth_image, 0)
            masked_depth = self.bridge.cv2_to_imgmsg(masked_depth, "passthrough")
            masked_depth.header = tmp_camera_info.header
            self.masked_depth_pub.publish(masked_depth)
            self.camera_info_pub.publish(tmp_camera_info)


if __name__ == '__main__':
    dd = DetectionMask()
    try:
        dd.process()
    except rospy.ROSInitException:
        pass
