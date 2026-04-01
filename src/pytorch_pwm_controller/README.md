# pytorch_pwm_controller

End-to-end imitation learning controller using a PyTorch Transformer model.
Subscribes to `/scan` (2D LiDAR) and publishes throttle + steering commands.

## Node: `nn_pwm_controller_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR scan (1081 beams) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/torch_pwm` | `geometry_msgs/TwistStamped` | `linear.x` = throttle PWM (µs), `angular.z` = steering angle (rad) |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `''` | Path to the trained `.pth` checkpoint file |
| `scan_topic` | `/scan` | LiDAR topic to subscribe |
| `pwm_topic` | `/torch_pwm` | Output topic name |
| `prediction_steps` | `1` | Number of future steps to predict (only step 0 is used) |
| `scan_history_length` | `10` | Number of past scans to stack as input sequence |
| `scan_history_stride` | `1` | Step interval between stacked scans |

### Model Architecture

```
scan sequence (seq_len × 1081)
      │
  Scan embedding (Linear → d_model=256)
      │
  Positional encoding
      │
  Transformer encoder (nhead=8, layers=3, ff_dim=512)
      │
  FC layers (256 → 128 → 64 → 2)
      │
 (throttle, steering_angle)
```

### Usage

```bash
ros2 run pytorch_pwm_controller nn_pwm_controller_node \
  --ros-args -p model_path:=/path/to/model.pth
```
