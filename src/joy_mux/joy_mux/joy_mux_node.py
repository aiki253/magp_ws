#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import json


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
        
        # Publishers
        self.mux_joy_pub = self.create_publisher(Joy, '/mux_joy', 10)
        self.status_pub = self.create_publisher(String, '/mux_joy/status', 10)
        
        # State variables
        self.manual_mode = True  # False: torch_joy (自動), True: joy (手動)
        self.last_ps_button_state = 0  # PlayStation button state
        self.latest_joy = None
        self.latest_torch_joy = None
        self.torch_joy_timeout = 1.0  # torch_joyのタイムアウト（秒）
        self.last_torch_joy_time = None
        
        # PlayStation button index (通常は12番, 環境により異なる場合あり)
        self.ps_button_index = 12
        # L2 button index (通常は8番, 環境により異なる場合あり)
        self.l2_button_index = 6
        
        self.get_logger().info('Joy Mux Node started. Press PlayStation button to toggle modes.')
        self.get_logger().info('Current mode: AUTO (torch_joy), fallback to joy if torch_joy unavailable')
        
        # 初期ステータスをパブリッシュ
        self.publish_status()
    
    def publish_status(self):
        """現在のモードステータスをJSON形式でパブリッシュ"""
        status_msg = String()
        status_dict = {
            'mode': 'Manual' if self.manual_mode else 'Auto',
            'source': 'joy' if self.manual_mode else 'torch_joy'
        }
        status_msg.data = json.dumps(status_dict)
        self.status_pub.publish(status_msg)
    
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
                
                # モード切り替え時にステータスをパブリッシュ
                self.publish_status()
            
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
                # L2ボタンが押されているかチェック
                l2_pressed = False
                if self.latest_joy is not None and len(self.latest_joy.buttons) > self.l2_button_index:
                    l2_pressed = self.latest_joy.buttons[self.l2_button_index] == 1
                
                if l2_pressed and self.latest_joy is not None:
                    # L2押下中: /joy と /torch_joy のaxes[0:3]を足し算
                    combined_msg = Joy()
                    combined_msg.header = self.latest_torch_joy.header
                    combined_msg.buttons = self.latest_torch_joy.buttons
                    combined_msg.axes = list(self.latest_torch_joy.axes)
                    
                    # axes[0:3]を足し算（最大・最小値でクリップ）
                    for i in range(min(4, len(combined_msg.axes), len(self.latest_joy.axes))):
                        combined_value = combined_msg.axes[i] + self.latest_joy.axes[i]
                        combined_msg.axes[i] = max(-1.0, min(1.0, combined_value))
                    
                    self.mux_joy_pub.publish(combined_msg)
                else:
                    # L2未押下: /torch_joy のみ
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