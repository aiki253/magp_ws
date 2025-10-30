#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float32
import json


class PwmMuxNode(Node):
    def __init__(self):
        super().__init__('pwm_mux_node')
        
        # パラメータの宣言
        self.declare_parameter('motor_neutral', 1620)
        self.declare_parameter('motor_min', 1100)
        self.declare_parameter('motor_max', 2300)
        self.declare_parameter('steering_center', 1500)
        self.declare_parameter('steering_left', 1200)
        self.declare_parameter('steering_right', 1800)
        self.declare_parameter('speed_deadzone', 0.05)
        self.declare_parameter('steering_deadzone', 0.05)
        self.declare_parameter('manual_gain', 1.0)
        self.declare_parameter('gain_step', 0.2)
        self.declare_parameter('ps_button_index', 12)
        self.declare_parameter('l2_button_index', 6)
        self.declare_parameter('left_stick_horizontal_axis', 0)
        
        # パラメータの取得
        self.MOTOR_NEUTRAL = self.get_parameter('motor_neutral').value
        self.MOTOR_MIN = self.get_parameter('motor_min').value
        self.MOTOR_MAX = self.get_parameter('motor_max').value
        self.STEERING_CENTER = self.get_parameter('steering_center').value
        self.STEERING_LEFT = self.get_parameter('steering_left').value
        self.STEERING_RIGHT = self.get_parameter('steering_right').value
        self.SPEED_DEADZONE = self.get_parameter('speed_deadzone').value
        self.STEERING_DEADZONE = self.get_parameter('steering_deadzone').value
        self.manual_gain = self.get_parameter('manual_gain').value
        self.gain_step = self.get_parameter('gain_step').value
        self.ps_button_index = self.get_parameter('ps_button_index').value
        self.l2_button_index = self.get_parameter('l2_button_index').value
        self.left_stick_horizontal_axis = self.get_parameter('left_stick_horizontal_axis').value
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.torch_pwm_sub = self.create_subscription(
            TwistStamped,
            '/torch_pwm',
            self.torch_pwm_callback,
            10
        )
        
        # Publishers
        self.mux_pwm_pub = self.create_publisher(TwistStamped, '/mux_pwm', 10)
        self.status_pub = self.create_publisher(String, '/mux_pwm/status', 10)
        self.manual_gain_pub = self.create_publisher(Float32, '/mux_pwm/manual_gain', 10)
        
        # State variables
        self.manual_mode = False  # False: Auto (torch_pwm), True: Manual (joy)
        self.last_ps_button_state = 0
        self.latest_joy = None
        self.latest_torch_pwm = None
        self.prev_left_stick_right = 0
        self.prev_left_stick_left = 0
        
        # 初期ステータスをパブリッシュ
        self.publish_status()
        self.publish_manual_gain()
    
    def convert_joy_to_motor_pwm(self, joy_value):
        """
        Joy値をモーターPWMに変換
        Args:
            joy_value: axes[1] の値 (-1.0 ~ +1.0)
        Returns:
            motor_pwm: PWM値（マイクロ秒）
        """
        if abs(joy_value) < self.SPEED_DEADZONE:
            return self.MOTOR_NEUTRAL
        elif joy_value > 0:
            # 前進: 1620 → 1100
            adjusted = (joy_value - self.SPEED_DEADZONE) / (1.0 - self.SPEED_DEADZONE)
            return self.MOTOR_NEUTRAL - (self.MOTOR_NEUTRAL - self.MOTOR_MIN) * adjusted
        else:
            # 後退: 1620 → 2300
            adjusted = (joy_value + self.SPEED_DEADZONE) / (1.0 - self.SPEED_DEADZONE)
            return self.MOTOR_NEUTRAL + (self.MOTOR_MAX - self.MOTOR_NEUTRAL) * abs(adjusted)
    
    def convert_joy_to_steering_pwm(self, joy_value):
        """
        Joy値をステアリングPWMに変換
        Args:
            joy_value: axes[2] の値 (-1.0 ~ +1.0)
        Returns:
            steering_pwm: PWM値（マイクロ秒）
        """
        if abs(joy_value) < self.STEERING_DEADZONE:
            return self.STEERING_CENTER
        elif joy_value > 0:
            # 右: 1500 → 1800
            return self.STEERING_CENTER + (self.STEERING_RIGHT - self.STEERING_CENTER) * joy_value
        else:
            # 左: 1500 → 1200
            return self.STEERING_CENTER + (self.STEERING_CENTER - self.STEERING_LEFT) * joy_value
    
    def apply_manual_addition(self, torch_pwm_msg, joy_msg):
        """
        手動加算: mux = manual_gain * (manual - neutral) + torch
        
        Args:
            torch_pwm_msg: TwistStamped（AI出力）
            joy_msg: Joy（手動入力）
        Returns:
            TwistStamped: 加算後のメッセージ
        """
        output_msg = TwistStamped()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.header.frame_id = "base_link"
        
        # Manual PWM値を計算
        manual_motor = self.convert_joy_to_motor_pwm(joy_msg.axes[1])
        manual_steering = self.convert_joy_to_steering_pwm(joy_msg.axes[2])
        
        # Torch PWM値を取得
        torch_motor = torch_pwm_msg.twist.linear.x
        torch_steering = torch_pwm_msg.twist.angular.z
        
        # ニュートラルからの差分を計算
        manual_motor_offset = manual_motor - self.MOTOR_NEUTRAL
        manual_steering_offset = manual_steering - self.STEERING_CENTER
        
        # 手動加算: mux = torch + gain * manual_offset
        mixed_motor = torch_motor + self.manual_gain * manual_motor_offset
        mixed_steering = torch_steering + self.manual_gain * manual_steering_offset
        
        # クリッピング
        output_msg.twist.linear.x = max(self.MOTOR_MIN, min(self.MOTOR_MAX, mixed_motor))
        output_msg.twist.angular.z = max(self.STEERING_LEFT, min(self.STEERING_RIGHT, mixed_steering))
        
        return output_msg
    
    def joy_callback(self, msg):
        """Joyメッセージのコールバック"""
        self.latest_joy = msg
        
        # PSボタンでのモード切り替え
        if len(msg.buttons) > self.ps_button_index:
            current_ps_state = msg.buttons[self.ps_button_index]
            if current_ps_state == 1 and self.last_ps_button_state == 0:
                self.manual_mode = not self.manual_mode
                self.publish_status()
            self.last_ps_button_state = current_ps_state
        
        # 左スティック横で手動ゲイン値調整
        if len(msg.axes) > self.left_stick_horizontal_axis:
            left_stick_horizontal = msg.axes[self.left_stick_horizontal_axis]
            
            # 右: ゲイン値を増やす
            if left_stick_horizontal > 0.5 and self.prev_left_stick_right == 0:
                self.manual_gain = min(1.0, self.manual_gain + self.gain_step)
                self.publish_manual_gain()
            
            # 左: ゲイン値を減らす
            if left_stick_horizontal < -0.5 and self.prev_left_stick_left == 0:
                self.manual_gain = max(0.0, self.manual_gain - self.gain_step)
                self.publish_manual_gain()
            
            # 前回の状態を更新
            self.prev_left_stick_right = 1 if left_stick_horizontal > 0.5 else 0
            self.prev_left_stick_left = 1 if left_stick_horizontal < -0.5 else 0
        
        # メッセージを発行
        self.publish_mux_pwm()
    
    def torch_pwm_callback(self, msg):
        """torch_pwmのコールバック"""
        self.latest_torch_pwm = msg
        self.publish_mux_pwm()
    
    def publish_mux_pwm(self):
        """現在のモードに応じてPWMデータを発行"""
        if self.manual_mode:
            # 手動モード
            if self.latest_joy is not None:
                output_msg = TwistStamped()
                output_msg.header.stamp = self.get_clock().now().to_msg()
                output_msg.header.frame_id = "base_link"
                output_msg.twist.linear.x = self.convert_joy_to_motor_pwm(self.latest_joy.axes[1])
                output_msg.twist.angular.z = self.convert_joy_to_steering_pwm(self.latest_joy.axes[2])
                self.mux_pwm_pub.publish(output_msg)
        else:
            # 自動モード
            if self.latest_torch_pwm is not None:
                # L2ボタンの状態をチェック
                l2_pressed = False
                if self.latest_joy is not None and len(self.latest_joy.buttons) > self.l2_button_index:
                    l2_pressed = self.latest_joy.buttons[self.l2_button_index] == 1
                
                if l2_pressed and self.latest_joy is not None:
                    # L2押下中: 手動入力を加算
                    output_msg = self.apply_manual_addition(self.latest_torch_pwm, self.latest_joy)
                else:
                    # L2未押下: torch_pwm をそのまま転送
                    output_msg = TwistStamped()
                    output_msg.header.stamp = self.get_clock().now().to_msg()
                    output_msg.header.frame_id = "base_link"
                    output_msg.twist = self.latest_torch_pwm.twist
                
                self.mux_pwm_pub.publish(output_msg)
        
        # ステータスをパブリッシュ
        self.publish_status()
    
    def publish_status(self):
        """現在のモード状態をJSON形式でパブリッシュ"""
        status_msg = String()
        
        # L2の状態を取得
        l2_pressed = False
        if self.latest_joy is not None and len(self.latest_joy.buttons) > self.l2_button_index:
            l2_pressed = self.latest_joy.buttons[self.l2_button_index] == 1
        
        status_dict = {
            'mode': 'Manual' if self.manual_mode else 'Auto',
            'source': 'joy' if self.manual_mode else 'torch_pwm',
            'l2_manual_addition': l2_pressed and not self.manual_mode,
            'manual_gain': float(self.manual_gain)
        }
        status_msg.data = json.dumps(status_dict)
        self.status_pub.publish(status_msg)
    
    def publish_manual_gain(self):
        """現在の手動ゲイン値をパブリッシュ"""
        gain_msg = Float32()
        gain_msg.data = float(self.manual_gain)
        self.manual_gain_pub.publish(gain_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PwmMuxNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()