import rclpy
import time
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64, Bool
from enum import Enum

class GO_LEFT_RIGHT(Enum):
    LEFT = 1
    RIGHT = 2
    NONE = 3

class TurnLeftNode(Node):
    def __init__(self):
        super().__init__('control_pack_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.publisher_done = self.create_publisher(Bool, '/parking_done', 1)
        self.twist_msg_ = Twist()
        self.angular_speed_ = 8.0
        
        self.parling_choice = GO_LEFT_RIGHT.NONE

        self.subscription = self.create_subscription(Int64, '/detect/parking_grid', self.parking_choice_callback, 10)
        self.subscription  # Prevent Python cleanup from deleting subscription

    def parking_choice_callback(self, msg):
        self.parling_choice = GO_LEFT_RIGHT(msg.data)
        self.get_logger().info('%d' % self.parling_choice.value)

        if self.parling_choice.value == GO_LEFT_RIGHT.LEFT.value:
            self.stop_turning(1.0)
            # self.go_forward(0.9)
            self.turn_front_left(1.0)
            self.go_forward(1.1)
            self.stop_turning(1.0)
            self.go_backward(1.2)
            self.turn_left(0.7)
            self.go_forward(1.0)

        elif self.parling_choice.value == GO_LEFT_RIGHT.RIGHT.value:
            self.stop_turning(1.0)
            # self.go_forward(0.5)
            self.turn_right(0.6)
            self.go_forward(1.5)
            self.stop_turning(1.1)
            self.go_backward(1.4)
            self.turn_right(0.6)
            self.go_forward(1.0)


            
        else:
            self.stop_turning(1.0)

        # rclpy.shutdown()
        parking_done_msg = Bool()
        parking_done_msg.data = True
        self.publisher_done.publish(parking_done_msg)
        raise SystemExit

    def go_forward(self, turn_duration):
        self.twist_msg_.linear.x = 0.2
        self.twist_msg_.angular.z = 0.0

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
        self.get_logger().info('Go forward completed.')

    def go_backward(self, turn_duration):
        self.twist_msg_.linear.x = -0.2
        self.twist_msg_.angular.z = 0.0

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
        self.get_logger().info('Go backward completed.')

    def turn_left(self, turn_duration):
        self.twist_msg_.linear.x = 0.0
        self.twist_msg_.angular.z = self.angular_speed_

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
        self.get_logger().info('Turn left completed.')

    def turn_right(self, turn_duration):
        self.twist_msg_.linear.x = 0.0
        self.twist_msg_.angular.z = -self.angular_speed_

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
        self.get_logger().info('Turn right completed.')

    def turn_front_left(self, turn_duration):
        self.twist_msg_.linear.x = 0.2
        self.twist_msg_.angular.z = self.angular_speed_

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
        self.get_logger().info('Turn left completed.')

    def turn_front_right(self, turn_duration):
        self.twist_msg_.linear.x = 0.2
        self.twist_msg_.angular.z = -self.angular_speed_

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.5)
        self.get_logger().info('Turn right completed.')
    
    def stop_turning(self, turn_duration):
        self.twist_msg_.linear.x = 0.0
        self.twist_msg_.angular.z = 0.0
        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.get_logger().info('Stop completed.')

def main(args=None):
    rclpy.init(args=args)
    turn_left_node = TurnLeftNode()
    try:
        rclpy.spin(turn_left_node)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
