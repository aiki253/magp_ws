#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
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
        
        # urg_node2を起動
        ExecuteProcess(
            cmd=['ros2', 'launch', 'urg_node2', 'urg_node2.launch.py'],
            output='screen'
        ),
        
        # joy_mux_nodeを起動
        Node(
            package='joy_mux',
            executable='joy_mux_node',
            name='joy_mux_node',
            output='screen'
        ),
        
        # joy_controller_nodeを起動
        Node(
            package='pytorch_joy_controller',
            executable='joy_controller_node',
            name='joy_controller_node',
            output='screen'
        ),
        
        # Pythonスクリプトを起動
        ExecuteProcess(
            cmd=['python3', f'{home_dir}/magp_ws/src/controller/joy_pwm_controller/joy_pwm_controller/joy_pca9685_controller.py'],
            output='screen'
        ),
        
        # bag_recorderを起動
        ExecuteProcess(
            cmd=['ros2', 'launch', 'bag_recorder', 'bag_recorder.launch.py'],
            output='screen'
        ),
        
        # m5stack_visualizerを起動
        ExecuteProcess(
            cmd=['ros2', 'launch', 'm5stack_visualizer', 'm5stack_visualizer.launch.py'],
            output='screen'
        ),
    ])