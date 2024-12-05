import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/rgb', 10)
        self.timer_ = self.create_timer(0.05, self.publish_frame)
        self.bridge_ = CvBridge()
        self.video_capture = cv2.VideoCapture('recorded_video.mp4')  # 替换成你的实际视频路径

    def publish_frame(self):
        ret, frame = self.video_capture.read()
        if ret:
            img_msg = self.bridge_.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(img_msg)
            self.get_logger().info('Published frame to /rgb')

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
