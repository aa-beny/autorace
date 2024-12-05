import time
import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64, Bool
from enum import Enum

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

from detect_interfaces.msg import BoundingBox  # Import the custom message type

class Mode(Enum):
    LANE = 1
    INTERSECTION = 2
    OBSTACLES = 3
    PARKING = 4

class GO_LEFT_RIGHT(Enum):
    LEFT = 1
    RIGHT = 2
    STRIGHT = 3

class node(Node):
    def __init__(self):
        super().__init__('core_node')
        self.subscription_signs = self.create_subscription(String, '/detect/signs', self.signs_callback, 1)
        self.subscription_traffic_light = self.create_subscription(String, '/detect/traffic_light', self.traffic_light_callback, 1)
        self.subscription_yellow_fraction = self.create_subscription(Int64, '/detect/yellow_fraction', self.yellow_fraction_callback, 1)
        self.subscription_white_fraction = self.create_subscription(Int64, '/detect/white_fraction', self.white_fraction_callback, 1)
        self.subscription_parking_done = self.create_subscription(Bool, '/parking_done', self.parking_done_callback, 1)
        self.subscription_avoidance_done = self.create_subscription(Bool, '/avoidance_done', self.avoidance_done_callback, 1)
        self.subscription_bounding_box = self.create_subscription(BoundingBox, '/detect/bounding_box', self.BoundingBox_callback, 1)

        self.subscription_signs
        self.subscription_traffic_light
        self.subscription_yellow_fraction
        self.subscription_white_fraction
        self.subscription_parking_done
        self.subscription_avoidance_done 

        self.publisher_which_line = self.create_publisher(Int64, '/detect/lane_mode', 1)
        self.pub_stop = self.create_publisher(Bool, '/control/go_stop', 1)
        self.pub_lane_toggle = self.create_publisher(Bool, '/detect/lane_toggle', 1)
        self.pub_avoidance = self.create_publisher(Bool, '/control/avoidance', 1)
        
        self.mode = Mode.LANE
        self.Turn = GO_LEFT_RIGHT.STRIGHT
        self.traffic_light = String()
        self.keep_going = False
        self.parking_done = False
        self.avoidance_done = False

        self.yellow_fraction = 30000 ##判斷在停車中 是否檢測到沒有黃線
        self.white_fraction = 0

        self.parking = False
        self.receive_error = False
        self.ready_to_out_ts = False
        self.ready_to_go_row = False
        self.receive_stop = False
        self.row_cnt = 0

        self.intersection_going = False

        self.cnt = 0

        self.row_stop = False

        self.xmin = 0
        self.ymin = 0
        self.xmax = 0
        self.ymax = 0

        self.now_time = datetime.datetime.now()
 
        self.parking_launch_ls = launch.LaunchService()
        parking_launch_description = load_launch('control', 'parking_launch')
        self.parking_launch_ls.include_launch_description(parking_launch_description)

        self.avoidance_launch_ls = launch.LaunchService()
        avoidance_launch_description = load_launch('control', 'avoidance_launch')
        self.avoidance_launch_ls.include_launch_description(avoidance_launch_description)

    def parking_done_callback(self, msg):
        self.parking_done = msg.data

        if self.parking_done == True:
            self.parking_launch_ls.shutdown()

    def BoundingBox_callback(self, msg):
        self.xmin = msg.xmin
        self.ymin = msg.ymin
        self.xmax = msg.xmax
        self.ymax = msg.ymax

    def avoidance_done_callback(self, msg):
        self.avoidance_done = msg.data

        if self.avoidance_done == True:
            self.avoidance_launch_ls.shutdown()
        
    def yellow_fraction_callback(self, msg):
        self.yellow_fraction = msg.data

    def white_fraction_callback(self, msg):
        self.white_fraction = msg.data

    def traffic_light_callback(self, msg):
        self.traffic_light = msg.data

    def signs_callback(self, msg):
        
        if msg.data == 'Ts':
            self.get_logger().info('Received: Intersection sign')

            # 切換模式為INTERSECTION
            self.mode = Mode.INTERSECTION

        elif msg.data == 'left':
            self.get_logger().info('Received: LEFT sign')
            
            if self.intersection_going == False:
                # go left flag
                self.Turn = GO_LEFT_RIGHT.LEFT

        elif msg.data == 'right':
            self.get_logger().info('Received: RIGHT sign')

            if self.intersection_going == False:
                # go right flag
                self.Turn = GO_LEFT_RIGHT.RIGHT

        elif msg.data == 'error':
            self.get_logger().info('Received: STOP sign')

            self.now_time = datetime.datetime.now()
            self.receive_error = True
            self.ready_to_out_ts = True
            self.intersection_going = True

        elif msg.data == 'dig':
            self.get_logger().info('Received: OBSTACLE sign %d' % int(self.mode.value))
            self.get_logger().info('change to obstacles mode')
            self.mode = Mode.OBSTACLES

        elif msg.data == 'park':
            self.get_logger().info('Received: PARKING sign')
            self.mode = Mode.PARKING

            self.intersection_going = False

        # elif msg.data == 'stop':
        #     self.get_logger().info('Received: STOPPPP sign')
            # if self.row_stop == True and self.stop_once == False:
            #     self.receive_stop = False
            # elif self.stop_once == True:
            #     self.receive_stop = True
            # self.receive_stop = True
        elif msg.data == 'row':
            self.get_logger().info('Received: STOPBAR sign')
            self.row_stop = True

            if self.xmax-self.xmin > 160:   #框框夠大時
            # if (self.xmax+self.xmin)/2 >=300 and (self.xmax+self.xmin)/2 <=400 and self.xmax-self.xmin > 240:
                # pub stop car
                stop_msg = Bool()
                stop_msg.data = True
                self.pub_stop.publish(stop_msg)
                self.row_cnt = 0
                self.get_logger().warning('0000000000000000')
                # self.ready_to_go_row = True
            else:
                self.get_logger().warning('efawefawef: TUNNEL sign')
                self.row_cnt = self.row_cnt+1 
            # else:
            #     # pub go car
            #     stop_msg = Bool()
            #     stop_msg.data = False
            #     self.pub_stop.publish(stop_msg)

        elif msg.data == 'cave':
            self.get_logger().info('Received: TUNNEL sign')
            
            # pug stop
            stop_msg = Bool()
            stop_msg.data = True
            self.pub_stop.publish(stop_msg)

        else:
            self.get_logger().info('Received: NONE sign')
            self.mode = self.mode
            self.receive_error = False
            # self.row_stop = False
            self.row_cnt = self.row_cnt+1 


        if self.row_cnt >= 30 and self.row_stop == True:
            stop_msg = Bool()
            stop_msg.data = False
            self.pub_stop.publish(stop_msg) 
            self.row_stop = False
            
        # out of Ts
        if self.receive_error == False and self.mode.value == Mode.INTERSECTION.value and self.ready_to_out_ts == True:
            end_time = datetime.datetime.now()
            self.intersection_going = True

            time_difference = end_time.timestamp() - self.now_time.timestamp()

            if abs(time_difference) >= 5.0:
                self.mode = Mode.LANE
                self.receive_error = False
                self.ready_to_out_ts = False

#==================================mode select===============================================
        if self.mode.value == Mode.LANE.value:
            self.get_logger().info('Mode: LANE')
            
            # initial variable
            self.Turn = GO_LEFT_RIGHT.STRIGHT

            # if self.ready_to_go_row == True:
            #     self.ready_to_go_row = False
            #     # pub go car
            #     stop_msg = Bool()
            #     stop_msg.data = False
            #     self.pub_stop.publish(stop_msg)

            # 紅綠燈關卡
            if self.keep_going == False:
                if self.traffic_light == "GREEN":
                    self.get_logger().info('GO')
                    self.keep_going = True
                    # pub go
                    stop_msg = Bool()
                    stop_msg.data = False
                    self.pub_stop.publish(stop_msg)
                else:
                    self.get_logger().info('STOP')
                    # pug stop
                    stop_msg = Bool()
                    stop_msg.data = True
                    self.pub_stop.publish(stop_msg)

            #publish to go_single_line -> 雙循線
            msg = Int64()
            msg.data = 0
            self.publisher_which_line.publish(msg)

        elif self.mode.value == Mode.INTERSECTION.value:
            self.get_logger().info('Mode: INTERSECTION')

            # if go left: pub go_single_line -> 循黃線
            if self.Turn.value == GO_LEFT_RIGHT.LEFT.value:
                msg = Int64()
                msg.data = 1    # yellow line
                self.publisher_which_line.publish(msg)
            
            # if go right: pub go_single_line -> 循白線
            elif self.Turn.value == GO_LEFT_RIGHT.RIGHT.value:
                msg = Int64()
                msg.data = 2    #white line
                self.publisher_which_line.publish(msg)

        elif self.mode.value == Mode.OBSTACLES.value:
            self.get_logger().info('Mode: OBSTACLES')
            #publish to go_single_line -> 雙循線
            msg = Int64()
            msg.data = 0
            self.publisher_which_line.publish(msg)
            
            ##==========================way1=================
            # publish to go_single_line -> 雙白線
            # msg = Int64()
            # msg.data = 2
            # self.publisher_which_line.publish(msg)

            # # 執行避障程式
            # self.avoidance_launch_ls.run()

            # ## detect lane go on
            # pub_lane_msg = Bool()
            # pub_lane_msg.data = True
            # self.pub_lane_toggle.publish(pub_lane_msg)

            # #切回循線模式s
            # self.mode = Mode.LANE
            #================================================

            # ##======================way2=====================
            avoidance_msg = Bool()
            avoidance_msg.data = True
            self.pub_avoidance.publish(avoidance_msg)
            # ##==============================================

        elif self.mode.value == Mode.PARKING.value:
            self.get_logger().info('Mode: PARKING')

            # ##======================way2:finish avoidance=====================
            avoidance_msg = Bool()
            avoidance_msg.data = False
            self.pub_avoidance.publish(avoidance_msg)
            # ##==============================================

            msg = Int64()
            stop_msg = Bool()
            pub_lane_msg = Bool()

            if self.yellow_fraction > 4000:
                msg.data = 1    # yellow line
                self.publisher_which_line.publish(msg)
                self.parking = True

            if self.yellow_fraction < 500 and not self.parking_done and self.parking:

                ## detect lane shutdown
                pub_lane_msg.data = False
                self.pub_lane_toggle.publish(pub_lane_msg)

                # 執行停車程式
                self.parking_launch_ls.run()

                ## detect lane go on
                pub_lane_msg.data = True
                self.pub_lane_toggle.publish(pub_lane_msg)

            if self.Turn.value == GO_LEFT_RIGHT.LEFT.value and self.parking_done:

                if self.white_fraction > 4000:
                    self.mode = Mode.LANE

def load_launch(launch_package, launch_name):
    launch_description = launch.LaunchDescription()
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(launch_package), '/launch', '/'+launch_name+'.py'])
        )
    )

    return launch_description

def main(args=None):
    rclpy.init(args=args)
    core_node = node()
    rclpy.spin(core_node)
    core_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
