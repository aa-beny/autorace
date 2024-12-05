# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge,CvBridgeError
import cv2

import numpy as np
import argparse
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'Image',
            self.listener_callback,
            100)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String ,'traffic_light' ,10)
        self.timer = self.create_timer(0.5, self.green_callback)
    def listener_callback(self, msg , save_img=False):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            (rows,cols,channels) = self.cv_image.shape
            if cols > 60 and rows > 60 :
                cv2.circle(self.cv_image, (50,50), 10, 255)
            cv2.imshow("Image window", self.cv_image)
            cv2.waitKey(100)
            self.cv_image = cv2.resize(self.cv_image,(320,180))
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
            cv2.imshow('hsv',hsv)
            cv2.waitKey(1)
            # # 遮罩
            G_Low = [25,52,72]
            G_high = [107,255,255]
            G_height = [114,99]
            G_width = [285,267]
            low_G = np.array(G_Low)
            up_G = np.array(G_high)
            mask_G = cv2.inRange(hsv,low_G,up_G)
            # output = cv2.bitwise_and(hsv, hsv, mask = mask_G )
            # roi_green = cv2.bitwise_and(roi, roi, mask=cv2.inRange(roi, (0, 100, 0), (100, 255, 100)))
            # green_mask = cv2.inRange(self.cv_image, np.array([0, 100, 0]), np.array([100, 255, 100]))
            for i in range(180):
                for j in range(320):
                    if i>G_height[0] or i<G_height[1] or j>G_width[0] or j<G_width[1]:
                        mask_G[i][j] = 0
            self.brightness = mask_G.sum()
            cv2.imshow("GREEN", mask_G)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def green_callback(self):
        if self.brightness > 7000:
            msg = String()
            msg.data = 'GREEN'
            self.publisher.publish(msg)
            self.get_logger().info("Publisher : GREEN")
        else:
            msg = String()
            msg.data = 'RED'
            self.publisher.publish(msg)
            self.get_logger().info("Publisher : RED")



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CameraPublisher()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
