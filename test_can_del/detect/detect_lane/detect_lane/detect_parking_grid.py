import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64, Bool
import math
from enum import Enum

class GO_LEFT_RIGHT(Enum):
    LEFT = 1
    RIGHT = 2
    NONE = 3

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('detect_parking_grid_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.subscription_parking_done = self.create_subscription(Bool, '/parking_done', self.parking_done_callback, 1)
        self.subscription  # Prevent Python cleanup from deleting subscription

        self.publisher_which_block = self.create_publisher(Int64, '/detect/parking_grid', 1)

        self.PARKING_CHOSE = GO_LEFT_RIGHT.NONE

    def parking_done_callback(self, msg):
        if msg.data == True:
            raise SystemExit

    def scan_callback(self, msg):
        # Convert angles to radians
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        index_90 = int((math.radians(100-180) - angle_min) / angle_increment)
        index_135 = int((math.radians(170-180) - angle_min) / angle_increment)
        index_minus45 = int((math.radians(-100+180) - angle_min) / angle_increment)
        index_minus90 = int((math.radians(-170+180) - angle_min) / angle_increment)

        # print(msg.ranges[index_90])

        # # Check if indices are within a reasonable range
        if 0 <= index_90 < len(msg.ranges) and 0 <= index_135 < len(msg.ranges):
            # Get distance values within the range of 45 to 135 degrees
            distances_45_to_135 = msg.ranges[index_90:index_135 + 1]
            # print(distances_45_to_135)

            # Check if any distance is less than 40 centimeters
            obstacle_detected_45_to_135 = any(distance < 0.6 for distance in distances_45_to_135)
            if obstacle_detected_45_to_135:
                self.get_logger().info("RIGHT OBSTACLE")
                self.PARKING_CHOSE = GO_LEFT_RIGHT.LEFT
                # self.get_logger().info("RIGHT OBSTACLE %d" %GO_LEFT_RIGHT.LEFT)
            # else:
            #     self.get_logger().info("RIGHT EMPTY")
            #     self.PARKING_CHOSE = GO_LEFT_RIGHT.RIGHT

        # #嫂的範圍好像不對
        # # Check if indices are within a reasonable range
        if 0 <= index_minus45 < len(msg.ranges) and 0 <= index_minus90 < len(msg.ranges):
            # Get distance values within the range of -45 to -135 degrees
            distances_minus45_to_minus135 = msg.ranges[index_minus90:index_minus45 + 1]
            # print(distances_minus45_to_minus135)

            # Check if any distance is less than 40 centimeters
            obstacle_detected_minus45_to_minus135 = any(distance < 0.6 for distance in distances_minus45_to_minus135)
            if obstacle_detected_minus45_to_minus135:
                self.get_logger().info("LEFT OBSTACLE")
                self.PARKING_CHOSE = GO_LEFT_RIGHT.RIGHT
                #self.get_logger().info("RIGHT OBSTACLE %d" %GO_LEFT_RIGHT.LEFT)
            # else:
            #     self.get_logger().info("LEFT EMPTY")
            #     self.PARKING_CHOSE = GO_LEFT_RIGHT.LEFT

        msg = Int64()
        msg.data = self.PARKING_CHOSE.value
        self.publisher_which_block.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scan_subscriber = ScanSubscriber()
    try:
        rclpy.spin(scan_subscriber)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()