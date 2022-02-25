# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import conditions
import platform 

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
        ),
        launch_ros.actions.Node(
            package='ros_msft_aibot', 
            executable='ros_msft_aibot', 
            name='ros_msft_aibot'),
#        launch_ros.actions.Node(
#            package='ros_qwiic_motor', 
#            executable='ros_qwiic_motor', 
#            name='ros_qwiic_motor'),
        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            name=['static_transform_publisher'],
            arguments=[
                '0.1', '0.2', '0.3', '0.4', '.5', '.6', 'map', 'camera'
            ])
    ])