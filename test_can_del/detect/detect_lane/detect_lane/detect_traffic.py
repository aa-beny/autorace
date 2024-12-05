#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import UInt8, Float64, Bool, Int64, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class detect(Node):
    def __init__(self):
        super().__init__('detect_node')

        # Declare parameters with default values
        self.declare_parameter('boundcenter_x', 480.0)
        self.declare_parameter('boundcenter_y', 320.0)
        self.declare_parameter('boundwidth', 50.0)
        self.declare_parameter('boundheight', 100.0)
        
        self.declare_parameter('hue_red_l', 0.0)
        self.declare_parameter('hue_red_h', 0.0)
        self.declare_parameter('saturation_red_l', 0.0)
        self.declare_parameter('saturation_red_h', 50.0)
        self.declare_parameter('lightness_red_l', 230.0)
        self.declare_parameter('lightness_red_h', 255.0)
        self.declare_parameter('reliability_red_line', 100.0)
        
        self.declare_parameter('hue_green_l', 8.0)
        self.declare_parameter('hue_green_h', 36.0)
        self.declare_parameter('saturation_green_l', 8.0)
        self.declare_parameter('saturation_green_h', 96.0)
        self.declare_parameter('lightness_green_l', 156.0)
        self.declare_parameter('lightness_green_h', 255.0)
        self.declare_parameter('reliability_green_line', 100.0)

        self.declare_parameter('calibration', False)   
        self.declare_parameter('bound_size', 50.0)   
        
        # Get parameter values
        self.boundcenter_x = int(self.get_parameter('boundcenter_x').value)
        self.boundcenter_y = int(self.get_parameter('boundcenter_y').value)
        self.boundwidth = int(self.get_parameter('boundwidth').value)
        self.boundheight = int(self.get_parameter('boundheight').value)

        self.hue_red_l = int(self.get_parameter('hue_red_l').value)
        self.hue_red_h = int(self.get_parameter('hue_red_h').value)
        self.saturation_red_l = int(self.get_parameter('saturation_red_l').value)
        self.saturation_red_h = int(self.get_parameter('saturation_red_h').value)
        self.lightness_red_l = int(self.get_parameter('lightness_red_l').value)
        self.lightness_red_h = int(self.get_parameter('lightness_red_h').value)
        self.reliability_red_line = int(self.get_parameter('reliability_red_line').value)

        self.hue_green_l = int(self.get_parameter('hue_green_l').value)
        self.hue_green_h = int(self.get_parameter('hue_green_h').value)
        self.saturation_green_l = int(self.get_parameter('saturation_green_l').value)
        self.saturation_green_h = int(self.get_parameter('saturation_green_h').value)
        self.lightness_green_l = int(self.get_parameter('lightness_green_l').value)
        self.lightness_green_h = int(self.get_parameter('lightness_green_h').value)
        self.reliability_green_line = int(self.get_parameter('reliability_green_line').value)

        self.calibration_mode = int(self.get_parameter('calibration').value)

        self.size = int(self.get_parameter('bound_size').value)**2

        ##sub
        self.subscription = self.create_subscription(Image, '/image/image_raw', self.image_callback, 1)

        ## pub
        self.publisher_green_fraction = self.create_publisher(Int64, '/detect/green_fraction', 1)
        self.publisher_red_fraction = self.create_publisher(Int64, '/detect/red_fraction', 1)
        self.publisher_traffic_light = self.create_publisher(String, '/detect/traffic_light', 1)

        # self.size = 50*50
        self.detect = False
        
        if self.calibration_mode == True:
            # pub
            self.publisher_green = self.create_publisher(Image, '/image/green_image', 1)
            self.publisher_red = self.create_publisher(Image, '/image/red_image', 1)
            self.pub_calib = self.create_publisher(Image, '/image/mask_calib',1)
            
            #sub variable
            self.subscription_boundcenter_x = self.create_subscription(Float64, '/detect/parameter/boundcenter_x', self.boundcenter_x_callback, 1)
            self.subscription_boundcenter_y = self.create_subscription(Float64, '/detect/parameter/boundcenter_y', self.boundcenter_ycallback, 1)
            self.subscription_boundwidth = self.create_subscription(Float64, '/detect/parameter/boundwidth', self.boundwidth_callback, 1)
            self.subscription_boundheight = self.create_subscription(Float64, '/detect/parameter/boundheight', self.boundheight_callback, 1)

            self.subscription_hue_red_l = self.create_subscription(Float64, '/detect/parameter/hue_red_l', self.hue_red_l_callback, 1)
            self.subscription_hue_red_h = self.create_subscription(Float64, '/detect/parameter/hue_red_h', self.hue_red_h_callback, 1)
            self.subscription_saturation_red_l = self.create_subscription(Float64, '/detect/parameter/saturation_red_l', self.saturation_red_l_callback, 1)
            self.subscription_saturation_red_h = self.create_subscription(Float64, '/detect/parameter/saturation_red_h', self.saturation_red_h_callback, 1)
            self.subscription_lightness_red_l = self.create_subscription(Float64, '/detect/parameter/lightness_red_l', self.lightness_red_l_callback, 1)
            self.subscription_lightness_red_h = self.create_subscription(Float64, '/detect/parameter/lightness_red_h', self.lightness_red_h_callback, 1)
            self.subscription_reliability_red_line = self.create_subscription(Float64, 'reliability_red_line', self.reliability_red_line_callback, 1)
            self.subscription_hue_green_l = self.create_subscription(Float64, '/detect/parameter/hue_green_l', self.hue_green_l_callback, 1)
            self.subscription_hue_green_h = self.create_subscription(Float64, '/detect/parameter/hue_green_h', self.hue_green_h_callback, 1)
            self.subscription_saturation_green_l = self.create_subscription(Float64, '/detect/parameter/saturation_green_l', self.saturation_green_l_callback, 1)
            self.subscription_saturation_green_h = self.create_subscription(Float64, '/detect/parameter/saturation_green_h', self.saturation_green_h_callback, 1)
            self.subscription_lightness_green_l = self.create_subscription(Float64, '/detect/parameter/lightness_green_l', self.lightness_green_l_callback, 1)
            self.subscription_lightness_green_h = self.create_subscription(Float64, '/detect/parameter/lightness_green_h', self.lightness_green_h_callback, 1)
            self.subscription_reliability_green_line = self.create_subscription(Float64, '/detect/parameter/reliability_green_line', self.reliability_green_line_callback, 1)

            self.subscription_boundsize = self.create_subscription(Float64, '/detect/parameter/boundsize', self.boundsize_callback, 1)

        self.cv_bridge = CvBridge()
    
    def boundcenter_x_callback(self, msg):
        self.boundcenter_x = int(msg.data)

    def boundcenter_ycallback(self, msg):
        self.boundcenter_y = int(msg.data)

    def boundwidth_callback(self, msg):
        self.boundwidth = int(msg.data)

    def boundheight_callback(self, msg):
        self.boundheight = int(msg.data)

    def hue_red_l_callback(self, msg):
        self.hue_red_l = int(msg.data)

    def hue_red_h_callback(self, msg):
        self.hue_red_h = int(msg.data)

    def saturation_red_l_callback(self, msg):
        self.saturation_red_l = int(msg.data)

    def saturation_red_h_callback(self, msg):
        self.saturation_red_h = int(msg.data)

    def lightness_red_l_callback(self, msg):
        self.lightness_red_l = int(msg.data)

    def lightness_red_h_callback(self, msg):
        self.lightness_red_h = int(msg.data)

    def reliability_red_line_callback(self, msg):
        self.reliability_red_line = int(msg.data)

    def hue_green_l_callback(self, msg):
        self.hue_green_l = int(msg.data)

    def hue_green_h_callback(self, msg):
        self.hue_green_h = int(msg.data)

    def saturation_green_l_callback(self, msg):
        self.saturation_green_l = int(msg.data)

    def saturation_green_h_callback(self, msg):
        self.saturation_green_h = int(msg.data)

    def lightness_green_l_callback(self, msg):
        self.lightness_green_l = int(msg.data)

    def lightness_green_h_callback(self, msg):
        self.lightness_green_h = int(msg.data)

    def reliability_green_line_callback(self, msg):
        self.reliability_green_line = int(msg.data)

    def boundsize_callback(self, msg):
        self.size = int(msg.data)**2

    def image_callback(self, msg):
        # 將ROS Image轉換成OpenCV格式
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Assuming your new image size is 640x480
        new_width, new_height = 640, 480
        # Resize the image
        cv_image_original = cv2.resize(cv_image, (new_width, new_height))
        ## ===================
        
        # 高斯濾波
        # cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        # 使用雙邊滤波
        cv_image_original = cv2.bilateralFilter(cv_image_original, d=9, sigmaColor=75, sigmaSpace=75)

        # Calculate the coordinates of the trapezoid
        top_x1, top_y1 = self.boundcenter_x - self.boundwidth // 2, self.boundcenter_y - self.boundheight // 2
        top_x2, top_y2 = self.boundcenter_x + self.boundwidth // 2, self.boundcenter_y - self.boundheight // 2
        bottom_x1, bottom_y1 = self.boundcenter_x - self.boundwidth // 2, self.boundcenter_y + self.boundheight // 2
        bottom_x2, bottom_y2 = self.boundcenter_x + self.boundwidth // 2, self.boundcenter_y + self.boundheight // 2

        lines = [(top_x1, top_y1, top_x2, top_y2),
                (top_x2, top_y2, bottom_x2, bottom_y2),
                (bottom_x2, bottom_y2, bottom_x1, bottom_y1),
                (bottom_x1, bottom_y1, top_x1, top_y1)]

        cv_image_calib = self.draw_lines(cv_image_original, lines, color=(0, 0, 255), thickness=2)

        # 創建一個黑色圖像，與原始圖像尺寸相同
        mask = np.zeros_like(cv_image_original)
        
        # 將四個頂點坐標組合成一個多邊形
        pts = np.array([[(top_x1, top_y1), (top_x2, top_y2), (bottom_x2, bottom_y2), (bottom_x1, bottom_y1)]], dtype=np.int32)

        # 將多邊形範圍內的部分填充為白色
        cv2.fillPoly(mask, pts, (255, 255, 255))

        # 將遮罩圖像與原始圖像進行按位 AND 運算，保留多邊形範圍內的部分
        masked_image = cv2.bitwise_and(cv_image_original, mask)

        # cv2.imshow('Detected', masked_image)
        # cv2.waitKey(1)

        if self.calibration_mode == True:
            self.pub_calib.publish(self.cv_bridge.cv2_to_imgmsg(cv_image_calib, encoding='bgr8'))

        red_fraction, cv_red_lane = self.maskredLane(masked_image)
        green_fraction, cv_green_lane = self.maskgreenLane(masked_image)

        self.get_logger().info('%d' % int(green_fraction))

         # Find contours in the masks
        contours_red, _ = cv2.findContours(cv_red_lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(cv_green_lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        light = String()
        # Draw rectangles around the detected areas
        for contour in contours_red:
            x, y, w, h = cv2.boundingRect(contour)

            if w*h < self.size and red_fraction < 50:
                self.detect = False
                continue

            self.detect = True
            light.data = "RED"
            self.publisher_traffic_light.publish(light)

            cv2.rectangle(cv_image_original, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(cv_image_original, 'RED', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        for contour in contours_green:
            x, y, w, h = cv2.boundingRect(contour)

            if w*h < self.size and green_fraction < 50:
                self.detect = False
                continue
            
            self.detect = True
            light.data = "GREEN"
            self.publisher_traffic_light.publish(light)
            
            cv2.rectangle(cv_image_original, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image_original, 'GREEN', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)        

        # 顯示結果
        cv2.imshow('Detected Circles', cv_image_original)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()

        print(red_fraction, green_fraction)

        if self.detect == False:
            light.data = ""
            self.publisher_traffic_light.publish(light)

    def draw_lines(self, image, lines, color, thickness):
        image_with_lines = np.copy(image)
        for line in lines:
            image_with_lines = cv2.line(image_with_lines, (line[0], line[1]), (line[2], line[3]), color, thickness)
        return image_with_lines

    def maskredLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_red_l
        Hue_h = self.hue_red_h
        Saturation_l = self.saturation_red_l
        Saturation_h = self.saturation_red_h
        Lightness_l = self.lightness_red_l
        Lightness_h = self.lightness_red_h

        lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_red = np.array([Hue_h, Saturation_h, Lightness_h])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)
        msg_red_line_reliability = UInt8()
        msg_red_line_reliability.data = self.reliability_red_line

        if self.calibration_mode == True:
            self.publisher_red.publish(self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        # self.publisher_red.publish(self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        return fraction_num, mask

    def maskgreenLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_green_l
        Hue_h = self.hue_green_h
        Saturation_l = self.saturation_green_l
        Saturation_h = self.saturation_green_h
        Lightness_l = self.lightness_green_l
        Lightness_h = self.lightness_green_h

        lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_green = np.array([Hue_h, Saturation_h, Lightness_h])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)

        msg_red_green_reliability = UInt8()
        msg_red_green_reliability.data = self.reliability_green_line

        if self.calibration_mode == True:
            self.publisher_green.publish(self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        # self.publisher_green.publish(self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        return fraction_num, mask

def main(args=None):
    rclpy.init(args=args)
    detect_node = detect()
    rclpy.spin(detect_node)
    detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()