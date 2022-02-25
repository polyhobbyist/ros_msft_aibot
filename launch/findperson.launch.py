# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch import conditions
import platform 

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    onnx_share_dir = get_package_share_directory('ros_msft_onnx')
    TinyYOLOv2ModelPath = os.path.join(
        onnx_share_dir,
        'models',
        'tinyyolov2-8.onnx')
    
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'onnx_model_path_arg', 
            default_value= TinyYOLOv2ModelPath,
            description="Onnx model path"),
        launch_ros.actions.Node(
            package='ros_msft_onnx', executable='ros_msft_onnx', output='screen',
            name=['ros_msft_onnx'],
            parameters=[
                {'onnx_model_path': launch.substitutions.LaunchConfiguration('onnx_model_path_arg')},
                {'link_name': 'camera'},
                {'confidence': 0.5},
                {'tensor_width': 416},
                {'tensor_height': 416},
                {'tracker_type': 'yolo'},
                {'image_topic': '/camera/color/image_raw'},
                {'debug': True}
            ]),
    ])