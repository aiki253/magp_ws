# M5Stack Visualizer

M5Stack Core2を使用したROS2 Bag RecorderとMux Input Statusのビジュアライザー

## パッケージ構造

```
ws/src/tools/m5stack_visualizer/
├── m5stack_visualizer/
│   ├── __init__.py
│   └── m5stack_bridge_node.py    # ROS2ブリッジノード
├── launch/
│   └── m5stack_visualizer.launch.py
├── platformio/                    # M5Stack用PlatformIOプロジェクト
│   ├── platformio.ini
│   └── src/
│       └── main.cpp
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── m5stack_visualizer
└── README.md
```

## セットアップ

### 1. ROS2パッケージのビルド

```bash
cd ~/ros2_ws/src/tools
git clone <your-repo> m5stack_visualizer  # または直接作成
cd ~/ros2_ws
colcon build --packages-select m5stack_visualizer
source install/setup.bash
```

### 2. 依存関係のインストール

```bash
# pyserialのインストール
pip3 install pyserial

# PlatformIOのインストール（未インストールの場合）
pip3 install platformio
```

### 3. M5Stackファームウェアのビルド＆アップロード

```bash
cd ~/ros2_ws/src/tools/m5stack_visualizer/platformio
pio run -t upload
```

### 4. シリアルポートの設定

```bash
# ポート確認
ls /dev/ttyUSB*

# パーミッション設定（再ログイン必要）
sudo usermod -a -G dialout $USER
```

## 使用方法

### 起動

```bash
# M5Stack Bridgeノードのみ起動
ros2 launch m5stack_visualizer m5stack_visualizer.launch.py serial_port:=/dev/ttyUSB0

# Bag Recorderと一緒に起動する場合
ros2 launch bag_recorder bag_recorder.launch.py &
ros2 launch m5stack_visualizer m5stack_visualizer.launch.py serial_port:=/dev/ttyUSB0
```

### 表示モード切り替え

M5Stack Core2のボタン:
- **BtnA (左)**: 次のモードへ切り替え
- **BtnB (中央)**: 画面リフレッシュ
- **BtnC (右)**: 前のモードへ切り替え

### 3つの表示モード

1. **MODE 0 (デフォルト): Mux Only Large**
   - /mux_input/statusを画面全体に大きく表示
   - Auto: 赤、Manual: 青、Error: 黄色

2. **MODE 1: Bag and Mux**
   - 上部: Bag Recorder状態
   - 下部: Mux Input状態

3. **MODE 2: Bag Only**
   - Bag Recorder状態を大きく表示
   - RECORDING: 赤、IDLE: 緑

## テスト

### Mux Input Statusのテスト

```bash
# JSON形式でPublish
ros2 topic pub /mux_input/status std_msgs/String "data: '{\"mode\": \"Auto\"}'" --once
ros2 topic pub /mux_input/status std_msgs/String "data: '{\"mode\": \"Manual\"}'" --once
ros2 topic pub /mux_input/status std_msgs/String "data: '{\"mode\": \"Error\"}'" --once

# テキスト形式でPublish
ros2 topic pub /mux_input/status std_msgs/String "data: 'Auto'" --once
```

### Bag Recorderのテスト

```bash
# Bag Recorderノードを起動
ros2 launch bag_recorder bag_recorder.launch.py

# PS3コントローラーで操作
# または手動でテスト
ros2 topic pub /bag_recorder/status std_msgs/String "data: '{\"state\": \"RECORDING\", \"system_unlocked\": true, \"current_bag\": \"test_bag\", \"bag_counter\": 1, \"recording_start_time\": \"2025-10-30T10:00:00\"}'" --once
```

## トラブルシューティング

### M5Stackに何も表示されない

1. シリアル接続を確認
```bash
ros2 run m5stack_visualizer m5stack_bridge_node --ros-args -p serial_port:=/dev/ttyUSB0 --log-level debug
```

2. M5StackのUSB接続を確認
```bash
ls -l /dev/ttyUSB*
```

### "Connection Lost"と表示される

```bash
# トピックの確認
ros2 topic list
ros2 topic echo /bag_recorder/status
ros2 topic echo /mux_input/status
```

### シリアルポートが開けない

```bash
# パーミッション確認
ls -l /dev/ttyUSB0

# グループ追加（要再ログイン）
sudo usermod -a -G dialout $USER

# 一時的な解決（再起動で元に戻る）
sudo chmod 666 /dev/ttyUSB0
```

## パラメータ

### Launch Parameters

- `serial_port`: M5StackのシリアルポートPath (default: /dev/ttyUSB0)
- `baud_rate`: ボーレート (default: 115200)

### 例

```bash
ros2 launch m5stack_visualizer m5stack_visualizer.launch.py serial_port:=/dev/ttyACM0 baud_rate:=115200
```