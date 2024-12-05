#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist

class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.cbFollowLane, 1)
        self.sub_stop = self.create_subscription(Bool, '/control/go_stop', self.cbStop, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        self.lastError = 0
        self.MAX_VEL = 0.22

        self.stop = False

    def cbStop(self, msg):
        self.stop = msg.data
        if self.stop == True:
            self.Stop_fun()

    def cbFollowLane(self, desired_center):
        print(self.stop)
        if self.stop == False:
            center = desired_center.data

            error = center - 320

            # true value
            Kp = 0.038 #0.17

            Kd = 0.06 #0.07

            angular_z = Kp * error + Kd * (error - self.lastError)
            self.lastError = error
            twist = Twist()
            twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 1200) ** 2.2), 0.20)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0) #11.0
            self.pub_cmd_vel.publish(twist)
        else:
            self.Stop_fun()

    def Stop_fun(self):
        self.get_logger().info("TurtleBot Stop")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist) 

def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
