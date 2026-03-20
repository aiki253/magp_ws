# pwm_controller

PCA9685 I2C PWM hardware driver node.
Receives blended PWM commands from `mux_pwm` and drives motor + servo via PCA9685.

## Node: `pwm_pca9685_controller`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mux_pwm` | `geometry_msgs/TwistStamped` | `linear.x` = throttle (µs), `angular.z` = steering (µs) |
| `/joy` | `sensor_msgs/Joy` | Joystick for speed limit and emergency stop |

### PWM Ranges

| Channel | Min | Max | Notes |
|---------|-----|-----|-------|
| Motor (throttle) | 1120 µs | 1920 µs | Neutral ~1620 µs |
| Steering | 1200 µs | 1800 µs | Center ~1640 µs |

### Joystick Controls (hardware-level)

| Button | Action |
|--------|--------|
| D-pad up | Increase speed limit |
| D-pad down | Decrease speed limit |
| L1 + R1 | Emergency stop (reverse thrust) |

### Hardware

- **IC**: PCA9685 16-channel 12-bit PWM driver
- **Interface**: I2C
- **Frequency**: 50 Hz (standard servo/ESC frequency)
- **Library**: `adafruit-circuitpython-pca9685`

### Dependencies

```bash
pip3 install adafruit-circuitpython-pca9685
```

Enable I2C on your SBC and connect PCA9685 to the I2C bus before launching.

### Usage

```bash
ros2 run pwm_controller pwm_pca9685_controller
```
