from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    calibration_arg = DeclareLaunchArgument('calibration', default_value='True')

    detect_lane_calib = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('detect'), '/launch', '/detect_lane_launch.py'
        ]),
        launch_arguments={'calibration': LaunchConfiguration('calibration')}.items(),
    )

    control_dual_xm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('control_montor'), '/launch', '/control_dual_xm_launch.py'
        ]),
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments and nodes to the launch description
    ld.add_action(calibration_arg)
    ld.add_action(control_dual_xm)
    ld.add_action(detect_lane_calib)

    return ld
