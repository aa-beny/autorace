#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import degrees

class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.cbFollowLane, 1)
        self.sub_stop = self.create_subscription(Bool, '/control/go_stop', self.cbStop, 1)
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)

        self.sub_avoidance = self.create_subscription(Bool, '/control/avoidance', self.avoidance_callback, 1)
        
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        self.lastError = 0
        self.MAX_VEL = 0.22
        self.MAV_LINE_VEL = 0.22
        self.MAV_ANGLE_VEL = 2.0
        self.kp = 0.038
        self.kd = 0.06

        self.avoidance_mode = False
        self.avoidance_max_distance = 0.48
        self.lidar_error = 0

        self.stop = False

    def cbStop(self, msg):
        self.stop = msg.data
        if self.stop == True:
            self.Stop_fun()

    def avoidance_callback(self, msg):
        if msg.data == True:
            self.avoidance_mode = True
            self.MAX_VEL = 0.18
            self.MAV_LINE_VEL = 0.155
            self.MAV_ANGLE_VEL = 1.6
            self.kp = 0.028
            self.kd = 0.06

        else:
            self.avoidance_mode = False
            self.MAX_VEL = 0.22
            self.MAV_LINE_VEL = 0.22
            self.MAV_ANGLE_VEL = 2.0
            self.kp = 0.038
            self.kd = 0.06

    def lidar_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        scan = msg.ranges
        lidar_error = 0
        for index, distance in enumerate(scan):
            angle = index*angle_increment+angle_min
            # print(angle,distance)
            
            angle_degrees = degrees(angle)
            # # 距離小於max_distance才加入參考
            # if distance<self.avoidance_max_distance:
            #     # print('hi i am in')
            #     if angle_degrees>-178 and angle_degrees<-100:
            #         # print(angle_degrees)
            #         lidar_error += (0.65*(self.avoidance_max_distance-distance))+(0.01*(-100-angle_degrees)) #0.03

            #     if angle_degrees<180 and angle_degrees>100:
            #         # print(angle_degrees)
            #         lidar_error -= (0.5*(self.avoidance_max_distance-distance))+(0.01*(angle_degrees-100))

            # 距離小於max_distance才加入參考
            if distance<self.avoidance_max_distance:
                # print('hi i am in')
                if angle_degrees>-178+180 and angle_degrees<-100+180:
                    # print(angle_degrees)
                    lidar_error += (0.66*(self.avoidance_max_distance-distance))+(0.01*(-100+180-angle_degrees)) #0.03
            if distance<0.35:
                if angle_degrees<180-180 and angle_degrees>110-180:
                    # print(angle_degrees)
                    lidar_error -= (0.5*(0.3-distance))+(0.01*(angle_degrees-110+180))

        if int(lidar_error) > 0:
            self.lidar_error = int(lidar_error*0.75)
        else:
            self.lidar_error = int(lidar_error*0.635)

    def cbFollowLane(self, desired_center):
        print(self.stop)
        if self.stop == False:
            center = desired_center.data

            if self.avoidance_mode == False:
                error = center - 320
            else:
                print('error', self.lidar_error)
                error = center - 320 + self.lidar_error

            # true value
            Kp = self.kp #0.17
            Kd = self.kd #0.07

            angular_z = Kp * error + Kd * (error - self.lastError)
            self.lastError = error
            twist = Twist()
            twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 1200) ** 2.2), self.MAV_LINE_VEL)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -max(angular_z, -self.MAV_ANGLE_VEL) if angular_z < 0 else -min(angular_z, self.MAV_ANGLE_VEL)
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