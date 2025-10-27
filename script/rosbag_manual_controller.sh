#!/bin/bash

echo "Starting Joy PWM Controller..."

# ROS2環境を設定
source /opt/ros/humble/setup.bash
source ~/magp_ws/install/setup.bash

# Joy nodeとPWM controllerを起動
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 &
sleep 1
ros2 launch urg_node2 urg_node2.launch.py &
sleep 1
python3 ~/magp_ws/src/controller/joy_pwm_controller/joy_pwm_controller/joy_pca9685_controller.py

# Ctrl+Cで両方のプロセスを停止