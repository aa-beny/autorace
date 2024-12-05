import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time
import math

class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('avoidance_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback,1)
        self.subscription 

        self.publisher_done = self.create_publisher(Bool, '/avoidance_done', 1)
        self.pub_lane_toggle = self.create_publisher(Bool, '/detect/lane_toggle', 1)

    def move_and_sleep(self, linear_speed, angular_speed, duration):
        msg_motor = Twist()
        msg_motor.linear.x = linear_speed
        msg_motor.angular.z = angular_speed
        self.publisher_.publish(msg_motor)
        time.sleep(duration)

    def scan_callback(self, msg):
        self.get_logger().info('aa')
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        index_minus178 = int((math.radians(-178) - angle_min) / angle_increment)
        index_minus170 = int((math.radians(-170) - angle_min) / angle_increment)

        pub_lane_msg = Bool()
        pub_done_msg = Bool()

        if 0 <= index_minus178 < len(msg.ranges) and 0 <= index_minus170 < len(msg.ranges):
            self.get_logger().info('in')
            distances_minus10_to_10 = msg.ranges[index_minus178:index_minus170 + 1]
            obstacle_detected_minus10_to_10 = any(distance < 0.35 for distance in distances_minus10_to_10)

            if obstacle_detected_minus10_to_10:
                #停止循線模式
                self.get_logger().info('STOP LANE')
                pub_lane_msg.data = False
                self.pub_lane_toggle.publish(pub_lane_msg)

                self.get_logger().info('Start Avoidance')
                self.move_and_sleep(0.0, 0.0, 1.2)   # linear_speed, angular_speed, duration
                self.move_and_sleep(0.1, 3.4, 0.6)
                self.move_and_sleep(0.2, 0.0, 1.2)
                self.move_and_sleep(0.1, -2.75, 0.7)
                self.move_and_sleep(0.2, 0.0, 1.7)
                self.move_and_sleep(0.1, -2.6, 0.5)
                self.move_and_sleep(0.2, 0.0, 1.2)
                self.move_and_sleep(0.1, 3.0, 0.6)
                self.move_and_sleep(0.0, 0.0, 0.1)

                pub_done_msg.data = True
                self.publisher_done.publish(pub_done_msg)
                raise SystemExit
    
def main(args=None):
    rclpy.init(args=args)
    avoidance_node = LaserScanSubscriber()
    try:
        rclpy.spin(avoidance_node)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    rclpy.shutdown()

if __name__ == '__main__':
    main()