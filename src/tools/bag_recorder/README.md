# Bag Recorder

ROS2でPS3コントローラーの/joyトピックを使用してrosbag記録を制御するパッケージです。

## 機能

- **Square（□）ボタン**: rosbag記録を開始
- **Circle（○）ボタン**: 記録を停止して保存
- **Cross（×）ボタン**: 記録を破棄
- **Triangle（△）ボタン**: システムアンロック（誤作動防止のためノード起動時3回押す必要あり）

## セットアップ

### 1. パッケージのビルド

```bash
cd ~/ros2_ws/src
# このディレクトリにbag_recorderパッケージを配置
cd ~/ros2_ws
colcon build --packages-select bag_recorder
source install/setup.bash
```

### 2. joyパッケージのインストール（未インストールの場合）

```bash
sudo apt install ros-humble-joy
```

## 使用方法

### 1. joyノードの起動

```bash
ros2 run joy joy_node
```

### 2. bag_recorderノードの起動

デフォルト設定で起動:
```bash
ros2 launch bag_recorder bag_recorder.launch.py
```

カスタムパラメータで起動:
```bash
ros2 launch bag_recorder bag_recorder.launch.py params_file:=/path/to/your/params.yaml
```

または直接実行:
```bash
ros2 run bag_recorder bag_recorder_node
```

### 3. 操作手順

1. ノード起動後、Triangleボタンを3回押してシステムをアンロック
2. Squareボタンで記録開始
3. Circleボタンで保存、またはCrossボタンで破棄

## パラメータ

| パラメータ名 | デフォルト値 | 説明 |
|-------------|-------------|------|
| record_all_topics | true | 全トピックを記録 |
| topic_list | [] | 記録するトピックのリスト |
| save_directory | ~/rosbag/ | 保存先ディレクトリ |
| disk_space_warning_threshold_gb | 10.0 | 警告閾値（GB） |
| disk_space_critical_threshold_gb | 3.0 | 記録不可閾値（GB） |
| button_square | 0 | Squareボタンのインデックス |
| button_cross | 1 | Crossボタンのインデックス |
| button_circle | 2 | Circleボタンのインデックス |
| button_triangle | 3 | Triangleボタンのインデックス |

## Publishされるトピック

### /bag_recorder/status (std_msgs/String)

JSON形式でステータスを1Hzで配信:
```json
{
  "state": "RECORDING",
  "system_unlocked": true,
  "current_bag": "rosbag2_2025_10_29-15_30_45_1",
  "recording_start_time": "2025-10-29T15:30:45.123456",
  "bag_counter": 1
}
```

### /bag_recorder/event (std_msgs/String)

JSON形式でイベントを配信:
```json
{
  "timestamp": "2025-10-29T15:30:45.123456",
  "event_type": "RECORDING_STARTED",
  "message": "rosbag2_2025_10_29-15_30_45_1"
}
```

イベントタイプ:
- SYSTEM_LOCKED
- UNLOCK_PROGRESS
- SYSTEM_UNLOCKED
- RECORDING_STARTED
- RECORDING_SAVED
- RECORDING_DISCARDED
- DISK_SPACE_WARNING
- DISK_SPACE_CRITICAL
- ERROR

## トラブルシューティング

### ボタンが反応しない

```bash
ros2 topic echo /joy
```
でボタンインデックスを確認し、パラメータを調整してください。

### 記録が開始されない

- ディスク容量を確認してください（3GB以上必要）
- `/bag_recorder/event`トピックでエラーメッセージを確認してください