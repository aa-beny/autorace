#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import UInt8, Float64, Bool, Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class detect(Node):
    def __init__(self):
        super().__init__('detect_node')

        # Declare parameters with default values
        self.declare_parameter('top_x', 640.0)
        self.declare_parameter('top_y', 0.0)
        self.declare_parameter('bottom_x', 640.0)
        self.declare_parameter('bottom_y', 360.0)
        
        self.declare_parameter('hue_white_l', 0.0)
        self.declare_parameter('hue_white_h', 0.0)
        self.declare_parameter('saturation_white_l', 0.0)
        self.declare_parameter('saturation_white_h', 50.0)
        self.declare_parameter('lightness_white_l', 230.0)
        self.declare_parameter('lightness_white_h', 255.0)
        self.declare_parameter('reliability_white_line', 100.0)
        
        self.declare_parameter('hue_yellow_l', 8.0)
        self.declare_parameter('hue_yellow_h', 36.0)
        self.declare_parameter('saturation_yellow_l', 8.0)
        self.declare_parameter('saturation_yellow_h', 96.0)
        self.declare_parameter('lightness_yellow_l', 156.0)
        self.declare_parameter('lightness_yellow_h', 255.0)
        self.declare_parameter('reliability_yellow_line', 100.0)

        self.declare_parameter('calibration', False)   
        
        # Get parameter values
        self.top_x = int(self.get_parameter('top_x').value)
        self.top_y = int(self.get_parameter('top_y').value)
        self.bottom_x = int(self.get_parameter('bottom_x').value)
        self.bottom_y = int(self.get_parameter('bottom_y').value)

        self.hue_white_l = int(self.get_parameter('hue_white_l').value)
        self.hue_white_h = int(self.get_parameter('hue_white_h').value)
        self.saturation_white_l = int(self.get_parameter('saturation_white_l').value)
        self.saturation_white_h = int(self.get_parameter('saturation_white_h').value)
        self.lightness_white_l = int(self.get_parameter('lightness_white_l').value)
        self.lightness_white_h = int(self.get_parameter('lightness_white_h').value)
        self.reliability_white_line = int(self.get_parameter('reliability_white_line').value)

        self.hue_yellow_l = int(self.get_parameter('hue_yellow_l').value)
        self.hue_yellow_h = int(self.get_parameter('hue_yellow_h').value)
        self.saturation_yellow_l = int(self.get_parameter('saturation_yellow_l').value)
        self.saturation_yellow_h = int(self.get_parameter('saturation_yellow_h').value)
        self.lightness_yellow_l = int(self.get_parameter('lightness_yellow_l').value)
        self.lightness_yellow_h = int(self.get_parameter('lightness_yellow_h').value)
        self.reliability_yellow_line = int(self.get_parameter('reliability_yellow_line').value)

        self.calibration_mode = int(self.get_parameter('calibration').value)

        MOV_AVG_LENGTH = 5
        self.lane_fit_bef = np.array([0, 0, 0])  # Initialize lane_fit_bef attribute   
        self.mov_avg_left = np.zeros((MOV_AVG_LENGTH, 3))
        self.mov_avg_right = np.zeros((MOV_AVG_LENGTH, 3))

        self.lane_toggle = True   
        self.go_single_line = 0 #0: dul 1: yellow 2: white
        self.avoidance_mode = False

        ##sub
        self.subscription = self.create_subscription(Image, '/image/image_raw', self.image_callback, 1)
        self.subscription_lane_toggle = self.create_subscription(Bool, '/detect/lane_toggle', self.lane_toggle_callback, 1)
        self.subscription_lane_mode = self.create_subscription(Int64, '/detect/lane_mode', self.lane_mode_callback, 1)

        self.sub_avoidance = self.create_subscription(Bool, '/control/avoidance', self.avoidance_callback, 1)

        ## pub
        self.publisher_lane = self.create_publisher(Image, '/detect/lane', 1)
        self.publisher_control_lane = self.create_publisher(Float64, '/control/lane', 1)
        self.publisher_yellow_fraction = self.create_publisher(Int64, '/detect/yellow_fraction', 1)
        self.publisher_white_fraction = self.create_publisher(Int64, '/detect/white_fraction', 1)
        
        if self.calibration_mode == True:
            # pub
            self.publisher_birdseye = self.create_publisher(Image, '/image/birdseye_image', 1)
            self.publisher_yellow = self.create_publisher(Image, '/image/yellow_image', 1)
            self.publisher_white = self.create_publisher(Image, '/image/white_image', 1)
            self.pub_calib = self.create_publisher(Image, '/image/image_calib',1)
            
            #sub variable
            self.subscription_top_x = self.create_subscription(Float64, '/detect/parameter/top_x', self.top_x_callback, 1)
            self.subscription_top_y = self.create_subscription(Float64, '/detect/parameter/top_y', self.top_y_callback, 1)
            self.subscription_bottom_x = self.create_subscription(Float64, '/detect/parameter/bottom_x', self.bottom_x_callback, 1)
            self.subscription_bottom_y = self.create_subscription(Float64, '/detect/parameter/bottom_y', self.bottom_y_callback, 1)
            self.subscription_hue_white_l = self.create_subscription(Float64, '/detect/parameter/hue_white_l', self.hue_white_l_callback, 1)
            self.subscription_hue_white_h = self.create_subscription(Float64, '/detect/parameter/hue_white_h', self.hue_white_h_callback, 1)
            self.subscription_saturation_white_l = self.create_subscription(Float64, '/detect/parameter/saturation_white_l', self.saturation_white_l_callback, 1)
            self.subscription_saturation_white_h = self.create_subscription(Float64, '/detect/parameter/saturation_white_h', self.saturation_white_h_callback, 1)
            self.subscription_lightness_white_l = self.create_subscription(Float64, '/detect/parameter/lightness_white_l', self.lightness_white_l_callback, 1)
            self.subscription_lightness_white_h = self.create_subscription(Float64, '/detect/parameter/lightness_white_h', self.lightness_white_h_callback, 1)
            self.subscription_reliability_white_line = self.create_subscription(Float64, 'reliability_white_line', self.reliability_white_line_callback, 1)
            self.subscription_hue_yellow_l = self.create_subscription(Float64, '/detect/parameter/hue_yellow_l', self.hue_yellow_l_callback, 1)
            self.subscription_hue_yellow_h = self.create_subscription(Float64, '/detect/parameter/hue_yellow_h', self.hue_yellow_h_callback, 1)
            self.subscription_saturation_yellow_l = self.create_subscription(Float64, '/detect/parameter/saturation_yellow_l', self.saturation_yellow_l_callback, 1)
            self.subscription_saturation_yellow_h = self.create_subscription(Float64, '/detect/parameter/saturation_yellow_h', self.saturation_yellow_h_callback, 1)
            self.subscription_lightness_yellow_l = self.create_subscription(Float64, '/detect/parameter/lightness_yellow_l', self.lightness_yellow_l_callback, 1)
            self.subscription_lightness_yellow_h = self.create_subscription(Float64, '/detect/parameter/lightness_yellow_h', self.lightness_yellow_h_callback, 1)
            self.subscription_reliability_yellow_line = self.create_subscription(Float64, '/detect/parameter/reliability_yellow_line', self.reliability_yellow_line_callback, 1)

        self.cv_bridge = CvBridge()
    
    def avoidance_callback(self, msg):
        if msg.data == True:
            self.avoidance_mode = True
        else:
            self.avoidance_mode = False

    def lane_toggle_callback(self, msg):
        self.lane_toggle = msg.data

    def lane_mode_callback(self, msg):
        self.go_single_line = msg.data

    def top_x_callback(self, msg):
        self.top_x = int(msg.data)

    def top_y_callback(self, msg):
        self.top_y = int(msg.data)

    def bottom_x_callback(self, msg):
        self.bottom_x = int(msg.data)

    def bottom_y_callback(self, msg):
        self.bottom_y = int(msg.data)

    def hue_white_l_callback(self, msg):
        self.hue_white_l = int(msg.data)

    def hue_white_h_callback(self, msg):
        self.hue_white_h = int(msg.data)

    def saturation_white_l_callback(self, msg):
        self.saturation_white_l = int(msg.data)

    def saturation_white_h_callback(self, msg):
        self.saturation_white_h = int(msg.data)

    def lightness_white_l_callback(self, msg):
        self.lightness_white_l = int(msg.data)

    def lightness_white_h_callback(self, msg):
        self.lightness_white_h = int(msg.data)

    def reliability_white_line_callback(self, msg):
        self.reliability_white_line = int(msg.data)

    def hue_yellow_l_callback(self, msg):
        self.hue_yellow_l = int(msg.data)

    def hue_yellow_h_callback(self, msg):
        self.hue_yellow_h = int(msg.data)

    def saturation_yellow_l_callback(self, msg):
        self.saturation_yellow_l = int(msg.data)

    def saturation_yellow_h_callback(self, msg):
        self.saturation_yellow_h = int(msg.data)

    def lightness_yellow_l_callback(self, msg):
        self.lightness_yellow_l = int(msg.data)

    def lightness_yellow_h_callback(self, msg):
        self.lightness_yellow_h = int(msg.data)

    def reliability_yellow_line_callback(self, msg):
        self.reliability_yellow_line = int(msg.data)

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

        top_width = self.top_x
        top_height = self.top_y
        bottom_width = self.bottom_x
        bottom_height = self.bottom_y

        # Calculate center coordinates
        center_x, center_y = new_width // 2, new_height // 2

        # Calculate the coordinates of the trapezoid
        top_x1, top_y1 = center_x - top_width // 2, top_height
        top_x2, top_y2 = center_x + top_width // 2, top_height
        bottom_x1, bottom_y1 = center_x - bottom_width // 2, bottom_height 
        bottom_x2, bottom_y2 = center_x + bottom_width // 2, bottom_height 

        # 鳥瞰圖畫線
        lines = [(top_x1, top_y1, top_x2, top_y2),
                (top_x2, top_y2, bottom_x2, bottom_y2),
                (bottom_x2, bottom_y2, bottom_x1, bottom_y1),
                (bottom_x1, bottom_y1, top_x1, top_y1)]

        cv_image_calib = self.draw_lines(cv_image_original, lines, color=(0, 0, 255), thickness=2)

        # Offset for moving the bird's-eye view down
        offset_y = 150

        # Updated destination points for perspective transformation (bird's-eye view)
        dst_pts = np.array([[0, offset_y], [new_width, offset_y], [new_width, new_height + offset_y], [0, new_height + offset_y]], dtype=np.float32)

        # Define source points for perspective transformation
        src_pts = np.array([[top_x1, top_y1], [top_x2, top_y2], [bottom_x2, bottom_y2], [bottom_x1, bottom_y1]], dtype=np.float32)

        # Calculate the perspective transformation matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)

        # Apply perspective transformation to the region of interest
        cv_birdseye_image = cv2.warpPerspective(cv_image_original, M, (new_width, new_height))

        if self.calibration_mode == True:
            self.pub_calib.publish(self.cv_bridge.cv2_to_imgmsg(cv_image_calib, encoding='bgr8'))
            self.publisher_birdseye.publish(self.cv_bridge.cv2_to_imgmsg(cv_birdseye_image, encoding='bgr8'))

        # 車道偵測
        cv_lane = np.copy(cv_birdseye_image)

        #============================將左右兩側塗黑===================
        # 複製cv_lane
        cv_lane_BLleft = cv_lane.copy()
        cv_lane_BLright = cv_lane.copy()

        # 獲取照片的寬度和高度
        cv_lane_height, cv_lane_width = cv_lane.shape[:2]

        # 設置要塗黑的區域
        start_x_left = 0
        end_x_left = int(cv_lane_width / 3)
        start_x_right = int(2 * cv_lane_width / 3)
        end_x_right = cv_lane_width

        # 塗黑左1/3的區域
        cv_lane_BLleft[:, start_x_left:end_x_left] = [0, 0, 0]

        # 塗黑右1/3的區域
        cv_lane_BLright[:, start_x_right:end_x_right] = [0, 0, 0]
        #===========================================================

        white_fraction, cv_white_lane = self.maskWhiteLane(cv_lane_BLleft)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_lane_BLright)

        rclpy.logging.get_logger('detect_node').info("white_fraction : %d" % white_fraction)
        rclpy.logging.get_logger('detect_node').info("yellow_fraction : %d" % yellow_fraction)
        yellow_fraction_msg = Int64()
        yellow_fraction_msg.data = yellow_fraction
        self.publisher_yellow_fraction.publish(yellow_fraction_msg)
        white_fraction_msg = Int64()
        white_fraction_msg.data = white_fraction
        self.publisher_white_fraction.publish(white_fraction_msg)

        if self.calibration_mode == False:
            #擬合車道線
            try:
                if yellow_fraction > 4000:
                    self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, cv_yellow_lane)
                    self.mov_avg_left = np.append(self.mov_avg_left,np.array([self.left_fit]), axis=0)

                if white_fraction > 4000:
                    self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, cv_white_lane)
                    self.mov_avg_right = np.append(self.mov_avg_right,np.array([self.right_fit]), axis=0)
            except:
                if yellow_fraction > 4000:
                    self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
                    self.mov_avg_left = np.array([self.left_fit])

                if white_fraction > 4000:
                    self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
                    self.mov_avg_right = np.array([self.right_fit])

            MOV_AVG_LENGTH = 5

            self.left_fit = np.array([np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])])
            self.right_fit = np.array([np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])])

            rclpy.logging.get_logger('detect_node').info("mov_avg_left")
            rclpy.logging.get_logger('detect_node').info(str(type(self.mov_avg_left)))

            if self.mov_avg_left.shape[0] > 1000:
                self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

            if self.mov_avg_right.shape[0] > 1000:
                self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]


            rclpy.logging.get_logger('detect_node').info("detect_lane")
            if self.go_single_line == 1:
                rclpy.logging.get_logger('detect_node').info("detect_yellow")
                self.make_yellow_lane(cv_lane, yellow_fraction)
            elif self.go_single_line == 2:
                rclpy.logging.get_logger('detect_node').info("detect_white")
                self.make_white_lane(cv_lane, white_fraction)
            else:
                rclpy.logging.get_logger('detect_node').info("detect_both")
                self.make_dul_lane(cv_lane, white_fraction, yellow_fraction)

    def draw_lines(self, image, lines, color, thickness):
        image_with_lines = np.copy(image)
        for line in lines:
            image_with_lines = cv2.line(image_with_lines, (line[0], line[1]), (line[2], line[3]), color, thickness)
        return image_with_lines

    def maskWhiteLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_white_l
        Hue_h = self.hue_white_h
        Saturation_l = self.saturation_white_l
        Saturation_h = self.saturation_white_h
        Lightness_l = self.lightness_white_l
        Lightness_h = self.lightness_white_h

        lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)
        if fraction_num > 15000: #30000
            self.hue_white_h-=5
            if self.lightness_white_l < 250:
                self.lightness_white_l += 5
        elif fraction_num < 5000:
            self.hue_white_h+=5
            if self.lightness_white_l > 50:
                self.lightness_white_l -= 5

        how_much_short = 0

        for i in range(0, 480):
            if np.count_nonzero(mask[i,::]) > 0:
                how_much_short += 1

        how_much_short = 480 - how_much_short

        if how_much_short > 250:
            if self.reliability_white_line >= 5:
                self.reliability_white_line -= 5
        elif how_much_short <= 250:
            if self.reliability_white_line <= 99:
                self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line

        if self.calibration_mode == True:
            self.publisher_white.publish(self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        return fraction_num, mask

    def maskYellowLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)
        if fraction_num > 15000: #30000
            if self.lightness_yellow_l < 250:
                self.lightness_yellow_l += 5
        elif fraction_num < 5000:
            if self.lightness_yellow_l > 50:
                self.lightness_yellow_l -= 5

        how_much_short = 0 #有多少行缺少白色或黃色像素的行數

        for i in range(0, 480):
            if np.count_nonzero(mask[i,::]) > 0:
                how_much_short += 1

        how_much_short = 480 - how_much_short

        if how_much_short > 250:
            if self.reliability_yellow_line >= 5:
                self.reliability_yellow_line -= 5
        elif how_much_short <= 250:
            if self.reliability_yellow_line <= 99:
                self.reliability_yellow_line += 5

        msg_white_yellow_reliability = UInt8()
        msg_white_yellow_reliability.data = self.reliability_yellow_line

        if self.calibration_mode == True:
            self.publisher_yellow.publish(self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        return fraction_num, mask

    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 150 #100
        lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) & (
        nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin)))

        # Again, extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        lane_fit = np.polyfit(y, x, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
            
        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int_(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 20

        # Set height of windows
        window_height = np.int_(img_w.shape[0] / nwindows)

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated for each window
        x_current = lane_base

        # Set the width of the windows +/- margin
        margin = 50

        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Create empty lists to receive lane pixel indices
        lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            # Identify the nonzero pixels in x and y within the window
            good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                nonzerox < win_x_high)).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_lane_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_lane_inds) > minpix:
                x_current = np.int_(np.mean(nonzerox[good_lane_inds]))

        # Concatenate the arrays of indices
        lane_inds = np.concatenate(lane_inds)

        # Extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except:
            lane_fit = self.lane_fit_bef

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit

    def make_dul_lane(self, cv_image, white_fraction, yellow_fraction):
        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))
        centerx = np.zeros(480, dtype=np.float64)

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        if yellow_fraction > 4000: #3000
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)

        if white_fraction > 4000: #3000
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)
        
        self.is_center_x_exist = True

        value = 240
        value_r = 240
        if self.avoidance_mode == True:
            value = 295
            value_r = 305
            # index = 200

        if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:   
            if white_fraction > 4000 and yellow_fraction > 4000: #3000
                print('hi')
                centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
                # print('==================================')
                # print(type(centerx))
                # rclpy.logging.get_logger('detect_node').info(str(type(centerx)))

                pts = np.hstack((pts_left, pts_right))
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

                # Draw the lane onto the warped blank image
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

            if white_fraction > 4000 and yellow_fraction <= 4000: #3000
                print('hi2')
                centerx = np.subtract(self.right_fitx, value_r) #320
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

            if white_fraction <= 4000 and yellow_fraction > 4000: #3000
                print('hi3')
                centerx = np.add(self.left_fitx, value) #320
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        elif self.reliability_white_line <= 50 and self.reliability_yellow_line > 50:
            print('hi4')
            centerx = np.add(self.left_fitx, value) #320
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        elif self.reliability_white_line > 50 and self.reliability_yellow_line <= 50:
            print('hi5')
            centerx = np.subtract(self.right_fitx, value_r) #320
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        else:
            print('hi6')
            self.is_center_x_exist = False
            # TODO: stop
            pass

        # Combine the result with the original image
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        # if self.is_center_x_exist == True:
        # # publishes lane center
        #     msg_desired_center = Float64()
        #     msg_desired_center.data = centerx.item(300) #240
        #     if self.lane_toggle == True:
        #         self.publisher_control_lane.publish(msg_desired_center)


        if np.any(centerx >= 480):
            centerx[300] = 480
        elif np.any(centerx <= 0):
            centerx[300] = 0


        if self.is_center_x_exist == True:
        # publishes lane center
            msg_desired_center = Float64()
            msg_desired_center.data = centerx.item(300)
            self.msg_desired_center = msg_desired_center.data

        else:
            msg_desired_center = Float64()
            msg_desired_center.data = self.msg_desired_center

        if self.lane_toggle == True:
            self.publisher_control_lane.publish(msg_desired_center)

        self.publisher_lane.publish(self.cv_bridge.cv2_to_imgmsg(final, encoding='bgr8'))
        
    def make_yellow_lane(self, cv_image, yellow_fraction):
        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))
        centerx = np.zeros(480, dtype=np.float64)

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        if yellow_fraction > 4000: #3000
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)

        self.is_center_x_exist = True

        if self.reliability_yellow_line > 50:
            print('hi4')
            centerx = np.add(self.left_fitx, 240) #320
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        else:
            print('hi6')
            self.is_center_x_exist = False
            # TODO: stop
            pass

        if np.any(centerx >= 480):
            centerx[300] = 480
        elif np.any(centerx <= 0):
            centerx[300] = 0

        # Combine the result with the original image
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        # if self.is_center_x_exist == True:
        # # publishes lane center
        #     msg_desired_center = Float64()
        #     msg_desired_center.data = centerx.item(280) 
        #     if self.lane_toggle == True:
        #         self.publisher_control_lane.publish(msg_desired_center)

        if self.is_center_x_exist == True:
        # publishes lane center
            msg_desired_center = Float64()
            msg_desired_center.data = centerx.item(300)
            self.msg_desired_center = msg_desired_center.data

        else:
            msg_desired_center = Float64()
            msg_desired_center.data = self.msg_desired_center

        if self.lane_toggle == True:
            self.publisher_control_lane.publish(msg_desired_center)

        self.publisher_lane.publish(self.cv_bridge.cv2_to_imgmsg(final, encoding='bgr8'))

    def make_white_lane(self, cv_image, white_fraction):
        rclpy.logging.get_logger('detect_node').info("line")
        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))
        centerx = np.zeros(480, dtype=np.float64)

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        if white_fraction > 4000: #3000
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)
        
        self.is_center_x_exist = True

        if self.reliability_white_line > 50:
            print('hi5')
            centerx = np.subtract(self.right_fitx, 240) #320
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        else:
            print('hi6')
            self.is_center_x_exist = False
            # TODO: stop
            pass

        if np.any(centerx >= 480):
            centerx[300] = 480
        elif np.any(centerx <= 0):
            centerx[300] = 0

        # Combine the result with the original image
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        # if self.is_center_x_exist == True:
        # # publishes lane center
        #     msg_desired_center = Float64()
        #     msg_desired_center.data = centerx.item(350) #230
        #     if self.lane_toggle == True:
        #         self.publisher_control_lane.publish(msg_desired_center)

        if self.is_center_x_exist == True:
        # publishes lane center
            msg_desired_center = Float64()
            msg_desired_center.data = centerx.item(300)
            self.msg_desired_center = msg_desired_center.data

        else:
            msg_desired_center = Float64()
            msg_desired_center.data = self.msg_desired_center

        if self.lane_toggle == True:
            self.publisher_control_lane.publish(msg_desired_center)

        self.publisher_lane.publish(self.cv_bridge.cv2_to_imgmsg(final, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    detect_node = detect()
    rclpy.spin(detect_node)
    detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()