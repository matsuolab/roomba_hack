#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from pytorchyolo import detect, models
import torch
import torchvision
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import cv2
import copy

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

    def callback_rgbd(self, data1, data2, data3):
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array

        self.camera_info = data3

    def process(self):
        path = "/root/roomba_hack/catkin_ws/src/three-dimensions_tutorial/yolov3/"

        # load category
        with open(path+"data/coco.names") as f:
            category = f.read().splitlines()

        # prepare model
        # model = models.load_model(path+"config/yolov3.cfg", path+"weights/yolov3.weights")
        model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=True)

        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue

            # inference
            tmp_rgb_image = copy.copy(self.rgb_image)
            tmp_depth_image = copy.copy(self.depth_image)
            tmp_camera_info = copy.copy(self.camera_info)

            model.eval()
            input_image_tensor = torch.from_numpy(tmp_rgb_image/255).float().permute(2,0,1)
            prediction = model([input_image_tensor])
            # boxes = prediction[0]["boxes"]
            masks = prediction[0]["masks"]


            # boxes = detect.detect_image(model, tmp_rgb_image)
            # [[x1, y1, x2, y2, confidence, class]]

            # depth_mask = np.zeros_like(tmp_depth_image)
            depth_mask = np.zeros_like(tmp_depth_image).astype("float32")

            seg_rgb_mask = np.zeros_like(tmp_rgb_image).astype("uint8")

            # # plot bouding box
            for index, mask in enumerate(masks):
                if(prediction[0]["scores"][index] <= 0.5):
                    break
                mask_numpy = mask.permute(1,2,0).repeat(1, 1, 3).detach().numpy()
                mask_2d_numpy = mask.permute(1,2,0).detach().numpy()

                # x1, y1, x2, y2 = map(int, box[:4])
                # cls_pred = int(box[5])
                # tmp_rgb_image = cv2.rectangle(tmp_rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                # tmp_rgb_image = cv2.putText(tmp_rgb_image, category[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                depth_mask[mask_2d_numpy.squeeze() >= 0.9] = 1
                seg_rgb_mask[mask_numpy >= 0.9] = 1


            # mask = masks[0].permute(1,2,0).repeat(1, 1, 3).detach().numpy()
            # print(tmp_rgb_image.dtype)
            # print("depth shape:" + depth_mask.shape)

            # depth_mask = masks[0].permute(1, 2, 0000000).detach().numpy()

            # mask float32
            # tmp_rgb_image unit8
            #tmp_rgb_image = tmp_rgb_image * (mask+0.5).astype("uint8")
            tmp_rgb_image = tmp_rgb_image * seg_rgb_mask

            
            
            tmp_rgb_image = cv2.cvtColor(tmp_rgb_image, cv2.COLOR_RGB2BGR)
            # unit8 only 
            detection_result = self.bridge.cv2_to_imgmsg(tmp_rgb_image, "bgr8")
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