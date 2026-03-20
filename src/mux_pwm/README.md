# mux_pwm

PWM multiplexer node that blends manual joystick input with autonomous model output.
Supports seamless switching between manual and autonomous modes.

## Node: `pwm_mux_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | PS3/PS4 joystick input |
| `/torch_pwm` | `geometry_msgs/TwistStamped` | Autonomous controller output |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mux_pwm` | `geometry_msgs/TwistStamped` | Blended PWM command sent to hardware |
| `/mux_status` | `std_msgs/String` | JSON: current mode (`manual` / `auto`) |
| `/manual_gain` | `std_msgs/Float32` | Manual input gain (0.0–1.0) |
| `/torch_gain` | `std_msgs/Float32` | Model output gain (0.0–1.0) |

### PWM Ranges

| Channel | Min | Neutral | Max |
|---------|-----|---------|-----|
| Motor (throttle) | 1100 µs | 1620 µs | 2300 µs |
| Steering | 1200 µs | 1640 µs | 1800 µs |

### Joystick Controls

| Button / Axis | Action |
|---------------|--------|
| PS button | Toggle manual / autonomous mode |
| L2 (held) | Enable manual blending in auto mode |
| L2 + D-pad up/down | Adjust manual gain |
| R2 + D-pad up/down | Adjust model (torch) gain |
| Left stick vertical | Throttle (manual mode) |
| Right stick horizontal | Steering (manual mode) |

### Usage

```bash
ros2 run mux_pwm pwm_mux_node
```
