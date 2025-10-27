#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

from .model_inference import Model


class JoyControllerNode(Node):
    def __init__(self):
        super().__init__('joy_controller_node')
        
        # パラメータの宣言
        self.declare_parameter('model_path', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('joy_topic', '/joy')
        
        # パラメータの取得
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        # モデルパスが指定されていない場合はデフォルトパスを使用
        if not model_path:
            package_share_dir = get_package_share_directory('pytorch_joy_controller')
            model_path = os.path.join(package_share_dir, 'model', 'model.pth')
        
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        
        self.get_logger().info(f'Loading model from: {model_path}')
        
        # モデルの初期化
        try:
            self.model = Model(model_path)
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
        
        # Publisher
        self.joy_pub = self.create_publisher(Joy, joy_topic, 10)
        
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Publishing to: {joy_topic}')
    
    def scan_callback(self, msg: LaserScan):
        try:
            # LaserScanデータをNumPy配列に変換
            scan_ranges = np.array(msg.ranges, dtype=np.float32)
            
            # inf値を最大値に、nan値を0に置換
            scan_ranges = np.where(np.isinf(scan_ranges), msg.range_max, scan_ranges)
            scan_ranges = np.where(np.isnan(scan_ranges), 0.0, scan_ranges)
            
            # 推論実行
            joy_axes = self.model.inference(scan_ranges)
            # print(joy_axes)
            
            # Joyメッセージの作成
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = "joy"
            joy_msg.axes = [0.0, 1.0, joy_axes[1]]
            # joy_msg.axes = [0.0, joy_axes[0], joy_axes[1]]
            joy_msg.buttons = []  # 必要に応じてボタンも追加可能
            
            # パブリッシュ
            self.joy_pub.publish(joy_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = JoyControllerNode()
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