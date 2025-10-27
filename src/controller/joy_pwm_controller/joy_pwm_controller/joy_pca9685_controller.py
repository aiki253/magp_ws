#!/usr/bin/env python3
"""
ROS2 Joy Controller with I2C PWM (PCA9685)
ROS2 Humble 対応
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from adafruit_pca9685 import PCA9685
import board
import busio

class JoyI2CController(Node):
    def __init__(self):
        super().__init__('joy_i2c_controller')
        
        # パラメータの宣言
        self.declare_parameter('motor_channel', 0)
        self.declare_parameter('steering_channel', 1)
        self.declare_parameter('i2c_address', 0x40)
        
        # パラメータの取得
        motor_ch = self.get_parameter('motor_channel').value
        steering_ch = self.get_parameter('steering_channel').value
        i2c_addr = self.get_parameter('i2c_address').value
        
        # I2C と PCA9685 の初期化
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=i2c_addr)
            self.pca.frequency = 50  # 50Hz (サーボ標準)
            self.get_logger().info(f'PCA9685 initialized at address 0x{i2c_addr:02x}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PCA9685: {e}')
            raise
        
        self.motor_channel = motor_ch
        self.steering_channel = steering_ch
        
        # PWM パラメータ（あなたの確定値）
        self.MOTOR_NEUTRAL = 1620
        self.MOTOR_MIN = 1500      # 後退最大
        self.MOTOR_MAX = 1900      # 前進最大
        
        self.STEERING_CENTER = 1500
        self.STEERING_LEFT = 1200   # 左最大
        self.STEERING_RIGHT = 1800  # 右最大
        
        # 現在の値
        self.current_speed = 0.0    # -1.0 ~ 1.0
        self.current_steering = 0.0 # -1.0 ~ 1.0
        
        # Joy入力の生の値を保存（速度制限適用前）
        self.raw_speed_input = 0.0
        self.raw_steering_input = 0.0
        
        # 速度制限（安全のため）
        self.max_speed_limit = 0.5  # 最大50%に制限（調整可能）
        
        # デッドゾーン設定
        self.speed_deadzone = 0.05    # スピード用デッドゾーン
        self.steering_deadzone = 0.05 # ステアリング用デッドゾーン
        
        # ESC 初期化
        self._initialize_esc()
        
        # Joy subscriber
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # タイマー（定期的な状態表示）
        self.timer = self.create_timer(1.0, self.status_callback)
        self.loop_count = 0
        
        self.get_logger().info('Joy I2C Controller initialized')
        self.get_logger().info('Left stick (horizontal): steering')
        self.get_logger().info('Right stick (vertical): speed')
        self.get_logger().info('Button A: neutral, Button B: stop')
        
    def _initialize_esc(self):
        """ESC を正しく初期化"""
        self.get_logger().info('Initializing ESC...')
        self.set_motor_speed(0)  # ニュートラルを送信
        self.set_steering_angle(0)  # ステアリング中央
        self.get_logger().info('ESC initialized')
    
    def _us_to_duty_cycle(self, microseconds):
        """マイクロ秒を duty cycle (0-65535) に変換"""
        pulse_length = 1000000 / self.pca.frequency
        return int(microseconds / pulse_length * 65535)
    
    def set_motor_speed(self, speed):
        """
        モーター速度を設定（デッドゾーン対応版）
        Args:
            speed: -1.0 ~ 1.0 (-1.0=後退最大, 0=停止, 1.0=前進最大)
        
        動作:
        - speed = 0: ニュートラル (1620μs)
        - speed > 0: ニュートラル → 前進最大 (1620 → 2000μs)
        - speed < 0: ニュートラル → 後退最大 (1620 → 1400μs)
        """
        speed = max(-1.0, min(1.0, speed))
        self.current_speed = speed
        
        # デッドゾーン処理
        if abs(speed) < self.speed_deadzone:
            # ニュートラル
            pulse_width = self.MOTOR_NEUTRAL
        elif speed > 0:
            # 前進: デッドゾーンを超えたら 1620 → 2000
            # speedを 0~1 の範囲に再マッピング
            adjusted_speed = (speed - self.speed_deadzone) / (1.0 - self.speed_deadzone)
            pulse_width = self.MOTOR_NEUTRAL + (self.MOTOR_MAX - self.MOTOR_NEUTRAL) * adjusted_speed
        else:
            # 後退: デッドゾーンを超えたら 1620 → 1400
            # speedを -1~0 の範囲に再マッピング
            adjusted_speed = (speed + self.speed_deadzone) / (1.0 - self.speed_deadzone)
            pulse_width = self.MOTOR_NEUTRAL + (self.MOTOR_NEUTRAL - self.MOTOR_MIN) * adjusted_speed
        
        duty_cycle = self._us_to_duty_cycle(pulse_width)
        self.pca.channels[self.motor_channel].duty_cycle = duty_cycle
    
    def set_steering_angle(self, angle):
        """
        ステアリング角度を設定
        Args:
            angle: -1.0 ~ 1.0 (-1.0=左最大, 0=中央, 1.0=右最大)
        """
        angle = max(-1.0, min(1.0, angle))
        self.current_steering = angle
        
        if angle > 0:
            # 右: 1500 → 1800
            pulse_width = self.STEERING_CENTER + (self.STEERING_RIGHT - self.STEERING_CENTER) * angle
        elif angle < 0:
            # 左: 1500 → 1200 (angle が負なのでそのまま掛ける)
            pulse_width = self.STEERING_CENTER + (self.STEERING_CENTER - self.STEERING_LEFT) * angle
        else:
            # 中央
            pulse_width = self.STEERING_CENTER
        
        duty_cycle = self._us_to_duty_cycle(pulse_width)
        self.pca.channels[self.steering_channel].duty_cycle = duty_cycle
    
    def joy_callback(self, msg):
        """Joyメッセージのコールバック"""
        try:
            # 軸のマッピング（標準的なゲームパッド）
            # 左スティック横: ステアリング (axis 0)
            # 右スティック縦: 速度 (axis 4 または 1)
            
            if len(msg.axes) >= 3:
                # ステアリング: 左スティック横 (axis 0; right horizontal stick)·
                # -1.0 = 左, +1.0 = 右
                steering_input = -msg.axes[2]  # 反転
                
                # デッドゾーン処理
                if abs(steering_input) < self.steering_deadzone:
                    steering_input = 0.0
                
                # 速度: 右スティック縦 (axis 1; left vertical stick)
                # -1.0 = 後退, +1.0 = 前進
                speed_input = msg.axes[1]  # 代替
                
                # 速度制限を適用
                speed_input = speed_input * self.max_speed_limit
                
                # 値を設定
                self.set_steering_angle(steering_input)
                self.set_motor_speed(speed_input)
            
            # ボタン処理
            if len(msg.buttons) > 0:
                # Button A (index 0): ニュートラル
                if msg.buttons[0]:
                    self.set_steering_angle(0)
                    self.set_motor_speed(0)
                    self.get_logger().info('Reset to neutral')
                
                # Button B (index 1): 緊急停止
                if msg.buttons[1]:
                    self.set_motor_speed(0)
                    self.get_logger().warn('Emergency stop')
                
                # Button X (index 2): 速度制限を上げる
                if len(msg.buttons) > 2 and msg.buttons[2]:
                    self.max_speed_limit = min(1.0, self.max_speed_limit + 0.1)
                    self.get_logger().info(f'Speed limit: {self.max_speed_limit:.1%}')
                
                # Button Y (index 3): 速度制限を下げる
                if len(msg.buttons) > 3 and msg.buttons[3]:
                    self.max_speed_limit = max(0.1, self.max_speed_limit - 0.1)
                    self.get_logger().info(f'Speed limit: {self.max_speed_limit:.1%}')
        
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {e}')
    
    def status_callback(self):
        """定期的な状態表示"""
        self.loop_count += 1
        if self.loop_count % 5 == 0:  # 5秒ごと
            # 実際のPWM値を計算
            if abs(self.current_speed) < self.speed_deadzone:
                pwm_value = self.MOTOR_NEUTRAL
            elif self.current_speed > 0:
                adjusted = (self.current_speed - self.speed_deadzone) / (1.0 - self.speed_deadzone)
                pwm_value = int(self.MOTOR_NEUTRAL + (self.MOTOR_MAX - self.MOTOR_NEUTRAL) * adjusted)
            else:
                adjusted = (self.current_speed + self.speed_deadzone) / (1.0 - self.speed_deadzone)
                pwm_value = int(self.MOTOR_NEUTRAL + (self.MOTOR_NEUTRAL - self.MOTOR_MIN) * adjusted)
            
            self.get_logger().info(
                f'Speed: {self.current_speed:+.2f} ({pwm_value}μs), '
                f'Steering: {self.current_steering:+.2f}, '
                f'Limit: {self.max_speed_limit:.1%}'
            )
    
    def cleanup(self):
        """クリーンアップ"""
        try:
            self.get_logger().info('Cleaning up...')
            self.set_motor_speed(0)
            self.set_steering_angle(0)
            self.pca.deinit()
            self.get_logger().info('Cleanup completed')
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')
    
    def __del__(self):
        self.cleanup()


def main(args=None):
    rclpy.init(args=args)
    
    controller = None
    try:
        controller = JoyI2CController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        if controller:
            controller.get_logger().info('Keyboard interrupt')
    except Exception as e:
        if controller:
            controller.get_logger().error(f'Error: {e}')
        else:
            print(f'Initialization error: {e}')
    finally:
        if controller:
            controller.cleanup()
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()