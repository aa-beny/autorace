import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/rgb',  # 更換為相應的ROS 2 topic名稱
            self.image_callback,
            10)
        self.subscription  # 避免subscription被垃圾回收
        self.cv_bridge = CvBridge()
        self.video_writer = None
        self.recording = False

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        cv2.imshow('Image', cv_image)
        key = cv2.waitKey(1)

        if key == ord('q') and not self.recording:
            self.start_recording(cv_image.shape[1], cv_image.shape[0])  # 寬度和高度
        elif key == ord('q') and self.recording:
            self.stop_recording()

        if self.recording:
            self.video_writer.write(cv_image)

    def start_recording(self, width, height):
        filename = 'recorded_video.mp4'
        codec = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(filename, codec, 25, (width, height))
        self.recording = True
        self.get_logger().info('Recording started.')

    def stop_recording(self):
        self.video_writer.release()
        self.recording = False
        self.get_logger().info('Recording stopped.')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
