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
        self.declare_parameter('motor_min', 1120)
        self.declare_parameter('motor_max', 1820)
        self.declare_parameter('motor_neutral', 1620)
        self.declare_parameter('speed_center', 1520)
        self.declare_parameter('speed_limit_lower', 1120)
        self.declare_parameter('speed_limit_upper', 1620)
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
        self.MOTOR_MIN_ABSOLUTE = self.get_parameter('motor_min').value
        self.MOTOR_MAX_ABSOLUTE = self.get_parameter('motor_max').value
        self.MOTOR_NEUTRAL = self.get_parameter('motor_neutral').value
        self.speed_CENTER = self.get_parameter('speed_center').value
        self.speed_LIMIT_LOWER = self.get_parameter('speed_limit_lower').value
        self.speed_LIMIT_UPPER = self.get_parameter('speed_limit_upper').value
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
        # 速度の最大値（十字キーで調整）
        self.current_speed_limit = self.speed_CENTER
        
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
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publisher
        self.pwm_input_pub = self.create_publisher(TwistStamped, '/pwm_input', 10)
        
        self.get_logger().info(f'初期速度リミット: {self.current_speed_limit}')
    
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
    
    def apply_speed_limit(self, input_motor_pwm):
        """
        入力PWM値に速度リミットを適用
        
        加速（前進）のみに適用し、バックは考慮しない
        
        Args:
            input_motor_pwm: 入力PWM値（μs）
        Returns:
            limited_motor_pwm: リミット適用後のPWM値（μs）
        """
        # 緊急停止時はニュートラルに固定
        if self.emergency_stop_active:
            return self.MOTOR_NEUTRAL
        
        # 前進（MOTOR_NEUTRALより小さい値）の場合のみリミットを適用
        if input_motor_pwm < self.MOTOR_NEUTRAL:
            # 速度リミットを適用
            limited_pwm = max(input_motor_pwm, self.current_speed_limit)
        else:
            # バック（MOTOR_NEUTRAL以下）の場合はリミットを適用しない
            limited_pwm = input_motor_pwm
        
        # 絶対的な範囲制限
        return max(self.MOTOR_MIN_ABSOLUTE, min(self.MOTOR_MAX_ABSOLUTE, limited_pwm))
    
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
    
    def joy_callback(self, msg):
        """Joyコールバック（緊急停止と速度リミット調整の両方を処理）"""
        # 緊急停止チェック
        self.emergency_stop_active = self.check_emergency_stop(msg)
        
        # 緊急停止中は速度リミット調整を無効化
        if self.emergency_stop_active:
            return
        
        # 十字キーによる速度リミット調整
        if len(msg.axes) > self.dpad_vertical_axis:
            dpad_vertical = msg.axes[self.dpad_vertical_axis]
            
            # 十字キー上: より上限値をあげる（より速い速度にできる）
            if dpad_vertical > 0.5 and self.prev_dpad_up == 0:
                new_limit = self.current_speed_limit - self.speed_scale_step
                
                # 下限を超えないようにチェック
                if new_limit >= self.speed_LIMIT_LOWER:
                    self.current_speed_limit = new_limit
                    self.get_logger().info(f'速度リミット: {self.current_speed_limit}')
                else:
                    self.get_logger().warn(f'速度リミット上限に到達: {self.speed_LIMIT_LOWER}')
            
            # 十字キー下: より上限値を下げる（より遅い速度に制限される）
            if dpad_vertical < -0.5 and self.prev_dpad_down == 0:
                new_limit = self.current_speed_limit + self.speed_scale_step
                
                # 上限を超えないようにチェック
                if new_limit <= self.speed_LIMIT_UPPER:
                    self.current_speed_limit = new_limit
                    self.get_logger().info(f'速度リミット: {self.current_speed_limit}')
                else:
                    self.get_logger().warn(f'速度リミット下限に到達: {self.speed_LIMIT_UPPER}')
            
            # 前回の状態を更新
            self.prev_dpad_up = 1 if dpad_vertical > 0.5 else 0
            self.prev_dpad_down = 1 if dpad_vertical < -0.5 else 0
    
    def mux_pwm_callback(self, msg):
        """/mux_pwm のコールバック"""
        output_msg = TwistStamped()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.header.frame_id = "base_link"
        
        # 速度リミット適用（緊急停止時は自動的にニュートラルに制限される）
        limited_motor = self.apply_speed_limit(msg.twist.linear.x)
        
        # 範囲チェック（二重チェック）
        output_msg.twist.linear.x = float(max(self.MOTOR_MIN_ABSOLUTE, min(self.MOTOR_MAX_ABSOLUTE, limited_motor)))
        output_msg.twist.angular.z = float(max(self.STEERING_LEFT, min(self.STEERING_RIGHT, msg.twist.angular.z)))
        
        # 緊急停止時はステアリングも中央に固定
        if self.emergency_stop_active:
            output_msg.twist.angular.z = float(self.STEERING_CENTER)
        
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