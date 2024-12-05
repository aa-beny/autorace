from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # control_lane_node = Node(
    #     package='control',
    #     executable='control_pack',
    # )

    control_lane_node = Node(
        package='control',
        executable='control_pack_2',
    )

    lidar_test_node = Node(
        package='detect_lane',
        executable='detect_parking_grid',
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments and nodes to the launch description

    ld.add_action(control_lane_node)
    ld.add_action(lidar_test_node)


    return ld


