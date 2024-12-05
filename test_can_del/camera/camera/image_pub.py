import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image/image_raw', 10)
        self.timer_ = self.create_timer(0.01, self.publish_image)
        self.bridge_ = CvBridge()

    def publish_image(self):
        img = cv2.imread('roll.png')
        # img = cv2.imread('horses.jpg')
        if img is not None:
            img_msg = self.bridge_.cv2_to_imgmsg(img, 'bgr8')
            self.publisher_.publish(img_msg)
            self.get_logger().info('Published image to /image/image_raw')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
