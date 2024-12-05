from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    calibration_arg = DeclareLaunchArgument('calibration', default_value='False')

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('detect'), '/launch', '/detect_lane_launch.py'
        ])
    )

    motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('control_montor'), '/launch', '/control_dual_xm_launch.py'
        ])
    )

    detect_lane_calib = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('detect'), '/launch', '/detect_lane_launch.py'
        ]),
        launch_arguments={'calibration': LaunchConfiguration('calibration')}.items(),
    )

    # signs_node = Node(
    #     package='detect',
    #     executable='detect_signs',
    # )

    signs_node = Node(
        package='detect',
        executable='key_pub_signs',
    )

    control_lane_node = Node(
        package='control',
        executable='control_lane',
    )

    core_node = Node(
        package='core',
        executable='core',
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments and nodes to the launch description
    ld.add_action(calibration_arg)
    ld.add_action(rplidar)
    ld.add_action(motor)
    ld.add_action(detect_lane_calib)
    ld.add_action(signs_node)
    ld.add_action(control_lane_node)
    ld.add_action(core_node)


    return ld


