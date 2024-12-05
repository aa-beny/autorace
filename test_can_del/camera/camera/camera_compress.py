import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/image/compressed', 10)
        self.timer = self.create_timer(0.001, self.publish_image)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error('Unable to open the camera.')
            raise RuntimeError('Unable to open the camera.')

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # Resize the frame to a smaller size if needed
                # resized_frame = cv2.resize(frame, (320, 240))
                # Encode the image to JPEG format
                _, jpeg_image = cv2.imencode('.jpg', frame)
                compressed_image_msg = CompressedImage()
                compressed_image_msg.format = 'jpeg'
                compressed_image_msg.data = jpeg_image.tobytes()
                self.publisher_.publish(compressed_image_msg)
                self.get_logger().info('Compressed image published to /image/compressed topic')
            except Exception as e:
                self.get_logger().error(f'Error converting and publishing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except Exception as e:
        print(f'Error during execution: {str(e)}')
    finally:
        if camera_publisher:
            camera_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
