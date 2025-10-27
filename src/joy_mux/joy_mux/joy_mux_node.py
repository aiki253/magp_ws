#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyMuxNode(Node):
    def __init__(self):
        super().__init__('joy_mux_node')
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.torch_joy_sub = self.create_subscription(
            Joy,
            '/torch_joy',
            self.torch_joy_callback,
            10
        )
        
        # Publisher
        self.mux_joy_pub = self.create_publisher(Joy, '/mux_joy', 10)
        
        # State variables
        self.manual_mode = False  # False: torch_joy (自動), True: joy (手動)
        self.last_ps_button_state = 0  # PlayStation button state
        self.latest_joy = None
        self.latest_torch_joy = None
        self.torch_joy_timeout = 1.0  # torch_joyのタイムアウト（秒）
        self.last_torch_joy_time = None
        
        # PlayStation button index (通常は12番, 環境により異なる場合あり)
        self.ps_button_index = 12
        
        self.get_logger().info('Joy Mux Node started. Press PlayStation button to toggle modes.')
        self.get_logger().info('Current mode: AUTO (torch_joy), fallback to joy if torch_joy unavailable')
    
    def joy_callback(self, msg):
        """手動操作用のジョイスティックからのコールバック"""
        self.latest_joy = msg
        
        # PlayStation buttonの状態をチェック
        if len(msg.buttons) > self.ps_button_index:
            current_ps_state = msg.buttons[self.ps_button_index]
            
            # ボタンが押された瞬間を検出（立ち上がりエッジ）
            if current_ps_state == 1 and self.last_ps_button_state == 0:
                self.manual_mode = not self.manual_mode
                mode_str = "MANUAL (joy)" if self.manual_mode else "AUTO (torch_joy)"
                self.get_logger().info(f'Mode switched to: {mode_str}')
            
            self.last_ps_button_state = current_ps_state
        
        # 現在のモードに応じてメッセージを発行
        self.publish_mux_joy()
    
    def torch_joy_callback(self, msg):
        """自動操作用のジョイスティックからのコールバック"""
        self.latest_torch_joy = msg
        
        # 現在のモードに応じてメッセージを発行
        self.publish_mux_joy()
    
    def publish_mux_joy(self):
        """現在のモードに応じて適切なジョイスティックデータを発行"""
        if self.manual_mode:
            # 手動モード: /joy のデータを使用
            if self.latest_joy is not None:
                self.mux_joy_pub.publish(self.latest_joy)
        else:
            # 自動モード: /torch_joy のデータを使用
            if self.latest_torch_joy is not None:
                self.mux_joy_pub.publish(self.latest_torch_joy)


def main(args=None):
    rclpy.init(args=args)
    node = JoyMuxNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()