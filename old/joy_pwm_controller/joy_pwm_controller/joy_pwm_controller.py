#!/usr/bin/env python3
from importlib.util import set_loader
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import Jetson.GPIO as GPIO

class JoyPWMController(Node):
    def __init__(self):
        super().__init__('joy_pwm_controller')
        
        # Joy subscriber
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # PWM parameters
        self.PWM_Hz = 70
        self.PWM_NeutralStr = 9.60  # @70Hz
        self.PWM_NeutralSpd = 10.80
        self.PWM_StopStr = 9.60
        self.PWM_StopSpd = 20.00
        self.Delta_Jog = 0.01
        self.spd_ref = self.PWM_NeutralSpd
        self.str_ref = self.PWM_NeutralStr
        
        # GPIO setup
        GPIO.setmode(GPIO.BOARD)
        self.gppin_str = 33
        self.gppin_acc = 32
        
        GPIO.setup(self.gppin_str, GPIO.OUT)
        self.servo_str = GPIO.PWM(self.gppin_str, self.PWM_Hz)
        self.servo_str.start(0)
        
        GPIO.setup(self.gppin_acc, GPIO.OUT)
        self.servo_acc = GPIO.PWM(self.gppin_acc, self.PWM_Hz)
        self.servo_acc.start(0)
        
        # Initialize servos to neutral
        self.servo_str.ChangeDutyCycle(self.PWM_NeutralStr)
        self.servo_acc.ChangeDutyCycle(self.PWM_NeutralSpd)
        
        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.update_servos)
        self.loop_count = 0
        
        self.get_logger().info('Joy PWM Controller initialized')
        self.get_logger().info('Left stick: speed, Right stick: stering')
        self.get_logger().info('Button mapping: A=neutral, B=stop')
        
    def joy_callback(self, msg):
        # Joystick axes mapping (standard gamepad)
        # Left stick horizontal (axis 0): steering
        # Right stick vertical (axis 4 or 3): speed
        
        if len(msg.axes) > 4:
            # Left stick horizontal for steering (inverted)
            steering_input = -msg.axes[2]  # -1 to 1
            # Right stick vertical for speed
            speed_input = msg.axes[1]  # -1 to 1 (up is positive)
            
            # Convert joystick input to PWM values
            # Steering: -1 = full left, +1 = full right
            steering_range = 2.0  # Total range for steering
            self.str_ref = self.PWM_NeutralStr + (steering_input * steering_range)
            
            # Speed: -1 = reverse, +1 = forward
            speed_range = 5.0  # Total range for speed
            self.spd_ref = self.PWM_NeutralSpd + (speed_input * speed_range)
            
        # Button handling
        if len(msg.buttons) > 0:
            # Button A (index 0): Reset to neutral
            if msg.buttons[0]:
                self.str_ref = self.PWM_NeutralStr
                self.spd_ref = self.PWM_NeutralSpd
                self.get_logger().info('Reset to neutral position')
            
            # Button B (index 1): Emergency stop
            if msg.buttons[1]:
                self.str_ref = self.PWM_StopStr
                self.spd_ref = self.PWM_StopSpd
                self.get_logger().warn('Emergency stop activated')
                
        # Limit PWM values to reasonable range
        self.str_ref = max(5.0, min(15.0, self.str_ref))
        self.spd_ref = max(5.0, min(15.0, self.spd_ref))
    
    def update_servos(self):
        # Update servo positions
        self.servo_str.ChangeDutyCycle(self.str_ref)
        self.servo_acc.ChangeDutyCycle(self.spd_ref)
        
        # Periodic logging
        self.loop_count += 1
        if self.loop_count % 10 == 0:  # Every 1 second
            self.get_logger().info(f'str={self.str_ref:.3f}, spd={self.spd_ref:.3f}')
    
    def __del__(self):
        self.cleanup()
    
    def cleanup(self):
        try:
            self.servo_str.stop()
            self.servo_acc.stop()
            GPIO.cleanup()
            self.get_logger().info('GPIO cleanup completed')
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    controller = JoyPWMController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt received')
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()