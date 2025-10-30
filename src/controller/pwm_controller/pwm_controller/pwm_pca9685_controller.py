#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from adafruit_pca9685 import PCA9685
import board
import busio


class PwmController(Node):
    def __init__(self):
        super().__init__('pwm_controller')
        
        # パラメータの宣言
        self.declare_parameter('motor_channel', 0)
        self.declare_parameter('steering_channel', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('motor_min', 1100)
        self.declare_parameter('motor_max', 2300)
        self.declare_parameter('motor_neutral', 1620)
        self.declare_parameter('steering_center', 1500)
        self.declare_parameter('steering_left', 1200)
        self.declare_parameter('steering_right', 1800)
        self.declare_parameter('speed_scale_step', 50)
        self.declare_parameter('l1_button_index', 4)
        self.declare_parameter('r1_button_index', 5)
        self.declare_parameter('dpad_vertical_axis', 7)
        
        # パラメータの取得
        self.motor_channel = self.get_parameter('motor_channel').value
        self.steering_channel = self.get_parameter('steering_channel').value
        i2c_addr = self.get_parameter('i2c_address').value
        self.MOTOR_MIN = self.get_parameter('motor_min').value
        self.MOTOR_MAX = self.get_parameter('motor_max').value
        self.MOTOR_NEUTRAL = self.get_parameter('motor_neutral').value
        self.STEERING_CENTER = self.get_parameter('steering_center').value
        self.STEERING_LEFT = self.get_parameter('steering_left').value
        self.STEERING_RIGHT = self.get_parameter('steering_right').value
        self.speed_scale_step = self.get_parameter('speed_scale_step').value
        self.l1_button_index = self.get_parameter('l1_button_index').value
        self.r1_button_index = self.get_parameter('r1_button_index').value
        self.dpad_vertical_axis = self.get_parameter('dpad_vertical_axis').value
        
        # I2C と PCA9685 の初期化
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=i2c_addr)
            self.pca.frequency = 50
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PCA9685: {e}')
            raise
        
        # State variables
        self.speed_scale_offset = 0
        self.emergency_stop_active = False
        self.current_motor_pwm = self.MOTOR_NEUTRAL
        self.current_steering_pwm = self.STEERING_CENTER
        self.prev_dpad_up = 0
        self.prev_dpad_down = 0
        
        # ESC初期化
        self._initialize_esc()
        
        # Subscribers
        self.mux_pwm_sub = self.create_subscription(
            TwistStamped,
            '/mux_pwm',
            self.mux_pwm_callback,
            10
        )
        self.joy_limit_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_limit_callback,
            10
        )
        self.joy_emergency_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_emergency_callback,
            10
        )
        
        # Publisher
        self.pwm_input_pub = self.create_publisher(TwistStamped, '/pwm_input', 10)
    
    def _initialize_esc(self):
        """ESCを正しく初期化"""
        self.set_motor_pwm(self.MOTOR_NEUTRAL)
        self.set_steering_pwm(self.STEERING_CENTER)
    
    def _us_to_duty_cycle(self, microseconds):
        """マイクロ秒を duty cycle (0-65535) に変換"""
        pulse_length = 1000000 / self.pca.frequency
        return int(microseconds / pulse_length * 65535)
    
    def set_motor_pwm(self, pwm_value):
        """
        モーターにPWM値を設定
        Args:
            pwm_value: PWM値（マイクロ秒）
        """
        duty_cycle = self._us_to_duty_cycle(pwm_value)
        self.pca.channels[self.motor_channel].duty_cycle = duty_cycle
    
    def set_steering_pwm(self, pwm_value):
        """
        ステアリングにPWM値を設定
        Args:
            pwm_value: PWM値（マイクロ秒）
        """
        duty_cycle = self._us_to_duty_cycle(pwm_value)
        self.pca.channels[self.steering_channel].duty_cycle = duty_cycle
    
    def apply_speed_scale(self, input_motor_pwm):
        """
        入力されたモーターPWM値にスケールオフセットを適用
        
        Args:
            input_motor_pwm: 入力PWM値（μs）
        Returns:
            scaled_motor_pwm: スケール適用後のPWM値（μs）
        """
        # ニュートラルからの差分を計算
        offset_from_neutral = input_motor_pwm - self.MOTOR_NEUTRAL
        
        # スケールオフセットを加算
        scaled_offset = offset_from_neutral + self.speed_scale_offset
        
        # ニュートラルに戻す
        scaled_motor_pwm = self.MOTOR_NEUTRAL + scaled_offset
        
        # 範囲制限
        return max(self.MOTOR_MIN, min(self.MOTOR_MAX, scaled_motor_pwm))
    
    def check_emergency_stop(self, joy_msg):
        """
        L1とR1が両方押されている間、緊急停止状態
        
        Returns:
            bool: 緊急停止が有効かどうか
        """
        if len(joy_msg.buttons) > max(self.l1_button_index, self.r1_button_index):
            l1_pressed = joy_msg.buttons[self.l1_button_index] == 1
            r1_pressed = joy_msg.buttons[self.r1_button_index] == 1
            return l1_pressed and r1_pressed
        return False
    
    def joy_emergency_callback(self, msg):
        """緊急停止監視用のJoyコールバック"""
        self.emergency_stop_active = self.check_emergency_stop(msg)
    
    def joy_limit_callback(self, msg):
        """十字キーによる速度スケール調整"""
        if len(msg.axes) > self.dpad_vertical_axis:
            dpad_vertical = msg.axes[self.dpad_vertical_axis]
            
            # 十字キー上: スケールを増やす
            if dpad_vertical > 0.5 and self.prev_dpad_up == 0:
                self.speed_scale_offset += self.speed_scale_step
            
            # 十字キー下: スケールを減らす
            if dpad_vertical < -0.5 and self.prev_dpad_down == 0:
                self.speed_scale_offset -= self.speed_scale_step
            
            # 前回の状態を更新
            self.prev_dpad_up = 1 if dpad_vertical > 0.5 else 0
            self.prev_dpad_down = 1 if dpad_vertical < -0.5 else 0
    
    def mux_pwm_callback(self, msg):
        """/mux_pwm のコールバック"""
        output_msg = TwistStamped()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.header.frame_id = "base_link"
        
        if self.emergency_stop_active:
            # 緊急停止中: ニュートラル/中央に固定
            output_msg.twist.linear.x = float(self.MOTOR_NEUTRAL)
            output_msg.twist.angular.z = float(self.STEERING_CENTER)
        else:
            # 通常動作: スケール適用
            scaled_motor = self.apply_speed_scale(msg.twist.linear.x)
            
            # 範囲チェック（二重チェック）
            output_msg.twist.linear.x = float(max(self.MOTOR_MIN, min(self.MOTOR_MAX, scaled_motor)))
            output_msg.twist.angular.z = float(max(self.STEERING_LEFT, min(self.STEERING_RIGHT, msg.twist.angular.z)))
        
        # 現在の値を保存
        self.current_motor_pwm = output_msg.twist.linear.x
        self.current_steering_pwm = output_msg.twist.angular.z
        
        # PWM出力
        self.set_motor_pwm(output_msg.twist.linear.x)
        self.set_steering_pwm(output_msg.twist.angular.z)
        
        # 可視化用にパブリッシュ
        self.pwm_input_pub.publish(output_msg)
    
    def cleanup(self):
        """クリーンアップ"""
        try:
            self.set_motor_pwm(self.MOTOR_NEUTRAL)
            self.set_steering_pwm(self.STEERING_CENTER)
            self.pca.deinit()
        except Exception as e:
            pass
    
    def __del__(self):
        self.cleanup()


def main(args=None):
    rclpy.init(args=args)
    
    controller = None
    try:
        controller = PwmController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if controller:
            controller.cleanup()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()