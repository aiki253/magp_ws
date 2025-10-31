#!/usr/bin/env python3
"""
Launch file for Joy I2C Controller
ROS2 Humble
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',  # ゲームパッドのデバイス
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Joy I2C Controller node
        Node(
            package='your_package_name',  # パッケージ名を変更
            executable='pwm_pca9685_controller.py',
            name='pwm_controller',
            output='screen',
            parameters=[{
                'motor_channel': 0,
                'steering_channel': 1,
                'i2c_address': 0x40,
            }]
        ),
    ])