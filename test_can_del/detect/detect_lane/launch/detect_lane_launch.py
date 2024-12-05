from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    calibration_arg = DeclareLaunchArgument('calibration', default_value='False')

    # Define the nodes
    # camera_node = Node(
    #     package='camera',
    #     executable='video_pub',
    # )

    # camera_node = Node(
    #     package='camera',
    #     executable='image_pub',
    # )

    camera_node = Node(
        package='camera',
        executable='H65_camera',
        parameters=[{'calibration': LaunchConfiguration('calibration')}]
    )
    
    # config=os.path.join(get_package_share_directory('detect_lane'), 'config', 'hsv_parameters_default.yaml')
    config=os.path.join(get_package_share_directory('detect_lane'), 'config', 'hsv_parameters_own.yaml')

    detect_node = Node(
        package='detect_lane',
        executable='detect_lane',
        parameters=[{'calibration': LaunchConfiguration('calibration')}, config],
    )

    hsv_node = Node(
        package='detect_lane',
        executable='hsv_param_adjustment',
        condition=IfCondition(LaunchConfiguration('calibration')),
    )

    rviz_calibraion_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('detect_lane'), 'config', 'calibration_mode.rviz')],
        condition=IfCondition(LaunchConfiguration('calibration')),
    )

    rviz_lane_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('detect_lane'), 'config', 'lane_mode.rviz')],
        condition=UnlessCondition(LaunchConfiguration('calibration')),
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments and nodes to the launch description
    ld.add_action(calibration_arg)
    ld.add_action(camera_node)
    ld.add_action(hsv_node)
    ld.add_action(detect_node)
    ld.add_action(rviz_calibraion_node)
    ld.add_action(rviz_lane_node)

    return ld


