#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import TwistStamped
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import time
from .model.model import Model


class NNControllerNode(Node):
    def __init__(self):
        super().__init__('nn_controller_node')
        self.start_time = time.time()
        
        # scan_rangesのシーケンスを保持するリスト
        self.scan_sequence = []
        
        # パラメータの宣言
        self.declare_parameter('model_path', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pwm_topic', '/torch_pwm')
        self.declare_parameter('prediction_steps', 1)
        self.declare_parameter('scan_history_length', 20)
        self.declare_parameter('scan_history_stride', 10)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('r2_button_index', 7)
        self.declare_parameter('button_square', 0)
        self.declare_parameter('button_cross', 1)
        self.declare_parameter('button_circle', 2)
        self.declare_parameter('button_triangle', 3)
        self.declare_parameter('model_square_path', '')
        self.declare_parameter('model_cross_path', '')
        self.declare_parameter('model_circle_path', '')
        self.declare_parameter('model_triangle_path', '')
        
        # パラメータの取得
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        prediction_steps = self.get_parameter('prediction_steps').get_parameter_value().integer_value
        self.scan_history_length = self.get_parameter('scan_history_length').get_parameter_value().integer_value
        self.scan_history_stride = self.get_parameter('scan_history_stride').get_parameter_value().integer_value
        self._prediction_steps = prediction_steps
        self.r2_idx = self.get_parameter('r2_button_index').get_parameter_value().integer_value
        self.btn_square = self.get_parameter('button_square').get_parameter_value().integer_value
        self.btn_cross = self.get_parameter('button_cross').get_parameter_value().integer_value
        self.btn_circle = self.get_parameter('button_circle').get_parameter_value().integer_value
        self.btn_triangle = self.get_parameter('button_triangle').get_parameter_value().integer_value
        self._prev_buttons = []
        
        # モデルパスが指定されていない場合はデフォルトパスを使用
        package_share_dir = get_package_share_directory('pytorch_pwm_controller')
        if not model_path:
            model_path = os.path.join(package_share_dir, 'model', 'model_v7.pth')
        def _resolve(param, default):
            p = self.get_parameter(param).get_parameter_value().string_value
            return p if p else os.path.join(package_share_dir, 'model', default)
        self._button_model_paths = {
            self.btn_square:   _resolve('model_square_path',   'model_v10_fine5.pth'),
            self.btn_cross:    _resolve('model_cross_path',    'model_v10_fine10.pth'),
            self.btn_circle:   _resolve('model_circle_path',   'model_v7.pth'),
            self.btn_triangle: _resolve('model_triangle_path', 'model_v6.pth'),
        }
        
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        pwm_topic = self.get_parameter('pwm_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        
        self.get_logger().info(f'Loading model from: {model_path}')
        self.get_logger().info(f'Prediction steps: {prediction_steps}')
        self.get_logger().info(f'Scan history: length={self.scan_history_length}, stride={self.scan_history_stride}')
        
        # モデルの初期化
        try:
            self.model = Model(
                model_path,
                prediction_steps=prediction_steps,
                scan_history_length=self.scan_history_length,
            )
            self.get_logger().info('Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            raise
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        
        # Publisher (TwistStamped for PWM commands)
        self.pwm_pub = self.create_publisher(TwistStamped, pwm_topic, 10)
        
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Publishing to: {pwm_topic}')
        self.get_logger().info(f'Subscribing to joy: {joy_topic}')

    def joy_callback(self, msg: Joy):
        buttons = list(msg.buttons)
        prev = self._prev_buttons
        r2 = len(buttons) > self.r2_idx and buttons[self.r2_idx] == 1
        if r2 and prev:
            for btn_idx, model_path in self._button_model_paths.items():
                prev_pressed = len(prev) > btn_idx and prev[btn_idx] == 1
                curr_pressed = len(buttons) > btn_idx and buttons[btn_idx] == 1
                if curr_pressed and not prev_pressed:
                    self._switch_model(model_path)
                    break
        self._prev_buttons = buttons

    def _switch_model(self, model_path: str):
        self.get_logger().info(f'Switching model to: {model_path}')
        try:
            self.model = Model(model_path, prediction_steps=self._prediction_steps,
                               scan_history_length=self.scan_history_length)
            self.get_logger().info(f'Model switched to: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to switch model: {str(e)}')

    def scan_callback(self, msg: LaserScan):
        try:
            # LaserScanデータをNumPy配列に変換
            scan_ranges = np.array(msg.ranges, dtype=np.float32)
            
            # inf値を最大値に、nan値を0に置換
            scan_ranges = np.where(np.isinf(scan_ranges), msg.range_max, scan_ranges)
            scan_ranges = np.where(np.isnan(scan_ranges), 0.0, scan_ranges)
            
            # シーケンスにscan_rangesを追加
            self.scan_sequence.append(scan_ranges)
            
            # 必要なデータ数を計算（strideを考慮）
            required_length = (self.scan_history_length - 1) * self.scan_history_stride + 1
            
            # 必要な数が溜まるまで待つ
            if len(self.scan_sequence) < required_length:
                return
            
            # 初回のみログ出力
            if len(self.scan_sequence) == required_length:
                self.get_logger().info(f'Ready for inference (scan: {required_length})')
            
            # 必要な数を超えたら古いものを削除
            while len(self.scan_sequence) > required_length:
                self.scan_sequence.pop(0)
            
            # スキャンデータをstrideを考慮して取得
            scan_indices = [i * self.scan_history_stride for i in range(self.scan_history_length)]
            inference_scan = np.stack(
                [self.scan_sequence[i] for i in scan_indices], axis=0
            )  # shape: (scan_history_length, scan_size)

            # 推論実行
            prediction = self.model.inference_next_step_only(inference_scan)

            # self.get_logger().info(f'DNN output is {prediction[0]:.4f}, {prediction[1]:.4f}')
            
            # 予測結果を取得 [throttle, angle]
            throttle = prediction[0]
            angle = prediction[1]
            
            # TwistStampedメッセージの作成
            pwm_msg = TwistStamped()
            pwm_msg.header.stamp = self.get_clock().now().to_msg()
            pwm_msg.header.frame_id = "base_link"
            
            # throttle
            pwm_msg.twist.linear.x = float(throttle)
            pwm_msg.twist.linear.y = 0.0
            pwm_msg.twist.linear.z = 0.0
            
            # angle
            pwm_msg.twist.angular.x = 0.0
            pwm_msg.twist.angular.y = 0.0
            pwm_msg.twist.angular.z = float(angle)
            
            # パブリッシュ
            self.pwm_pub.publish(pwm_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NNControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()