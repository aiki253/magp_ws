# Joy Mux

PlayStation®ボタンで手動/自動を切り替えられるジョイスティックマルチプレクサ

## 機能

- `/joy` と `/torch_joy` を受信
- PlayStation®ボタンで手動/自動モードを切り替え
- `/mux_joy` として統合されたジョイスティックデータを出力
- `/torch_joy` が利用できない場合は自動的に `/joy` にフォールバック

## 使用方法
```bash
ros2 run joy_mux joy_mux_node
```

## トピック

- **入力**:
  - `/joy` (sensor_msgs/Joy): 手動操作用ジョイスティック
  - `/torch_joy` (sensor_msgs/Joy): 自動操作用ジョイスティック

- **出力**:
  - `/mux_joy` (sensor_msgs/Joy): 統合されたジョイスティックデータ

## モード切り替え

PlayStation®ボタン（通常ボタンインデックス12）を押すことでモードを切り替えます。