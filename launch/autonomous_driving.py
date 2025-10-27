#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ホームディレクトリのパスを取得
    home_dir = os.path.expanduser('~')
    
    return LaunchDescription([
        # joy_nodeを起動
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0'
            }]
        ),
        
        # 1秒待機してからurg_node2を起動
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'urg_node2', 'urg_node2.launch.py'],
                    output='screen'
                )
            ]
        ),
        
        # 2秒待機してからjoy_mux_nodeを起動
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='joy_mux',
                    executable='joy_mux_node',
                    name='joy_mux_node',
                    output='screen'
                )
            ]
        ),
        
        # 3秒待機してからjoy_controller_nodeを起動
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='pytorch_joy_controller',
                    executable='joy_controller_node',
                    name='joy_controller_node',
                    output='screen'
                )
            ]
        ),
        
        # 4秒待機してからPythonスクリプトを起動
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', f'{home_dir}/magp_ws/src/controller/joy_pwm_controller/joy_pwm_controller/joy_pca9685_controller.py'],
                    output='screen'
                )
            ]
        ),
    ])