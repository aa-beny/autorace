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
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_done = self.create_publisher(Bool, '/parking_done', 1)
        self.twist_msg_ = Twist()
        self.angular_speed_ = 2.5
        self.go_forward(7.8)

        self.packing_left = False
        self.packing_right = True

        self.parling_choice = GO_LEFT_RIGHT.NONE

        self.subscription = self.create_subscription(Int64, '/parking_choice_topic', self.parking_choice_callback, 10)
        self.subscription  # Prevent Python cleanup from deleting subscription
        # if self.packing_left:
        #     self.timer_ = self.create_timer(1.0, self.packing_left_fun)

        # elif self.packing_right:
        #     self.timer_ = self.create_timer(1.0, self.packing_right_fun)

    def parking_choice_callback(self, msg):
        self.parling_choice = GO_LEFT_RIGHT(msg.data)
        self.get_logger().info('%d' % self.parling_choice.value)

        if self.parling_choice.value == GO_LEFT_RIGHT.LEFT.value:
            self.turn_left(7.8)
            self.go_forward(7.8)
            self.stop_turning(1.0)
            self.go_backward(7.8)
            self.turn_left(7.8)
            self.go_forward(7.8)

        elif self.parling_choice.value == GO_LEFT_RIGHT.RIGHT.value:
            self.turn_right(7.8)
            self.go_forward(7.8)
            self.stop_turning(1.0)
            self.go_backward(7.8)
            self.turn_right(7.8)
            self.go_forward(7.8)
        else:
            self.stop_turning(1.0)

        # rclpy.shutdown()
        parking_done_msg = Bool()
        parking_done_msg.data = True
        self.publisher_done.publish(parking_done_msg)
        raise SystemExit

    def packing_left_fun(self):
        self.go_forward(7.8)
        self.turn_left(7.8)
        self.go_forward(7.8)
        self.stop_turning(1.0)
        self.go_backward(7.8)
        self.turn_left(7.8)
        self.go_forward(7.8)

    def packing_right_fun(self):
        self.go_forward(7.8)
        self.turn_right(7.8)
        self.go_forward(7.8)
        self.stop_turning(1.0)
        self.go_backward(7.8)
        self.turn_right(7.8)
        self.go_forward(7.8)

    def go_forward(self, turn_duration):
        self.twist_msg_.linear.x = 50.0
        self.twist_msg_.angular.z = 0.0

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
        self.get_logger().info('Go forward completed.')

    def go_backward(self, turn_duration):
        self.twist_msg_.linear.x = -50.0
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
        self.twist_msg_.linear.x = 50.0
        self.twist_msg_.angular.z = self.angular_speed_

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
        self.get_logger().info('Turn left completed.')

    def turn_front_right(self, turn_duration):
        self.twist_msg_.linear.x = 50.0
        self.twist_msg_.angular.z = -self.angular_speed_

        self.publisher_.publish(self.twist_msg_)
        time.sleep(turn_duration)
        self.stop_turning(0.01)
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
