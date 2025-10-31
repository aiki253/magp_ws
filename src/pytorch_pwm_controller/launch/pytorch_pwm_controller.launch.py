#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # パッケージのパスを取得
    package_dir = get_package_share_directory('pytorch_pwm_controller')
    default_model_path = os.path.join(package_dir, 'model', 'model.pth')
    
    # Launch引数の定義
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path to the PyTorch model file'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Input laser scan topic'
    )
    
    joy_topic_arg = DeclareLaunchArgument(
        'joy_topic',
        default_value='/joy',
        description='Output joy topic'
    )
    
    # ノードの定義
    joy_controller_node = Node(
        package='pytorch_pwm_controller',
        executable='nn_controller_node',
        name='nn_controller_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'joy_topic': LaunchConfiguration('joy_topic'),
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        scan_topic_arg,
        joy_topic_arg,
        joy_controller_node,
    ])