# Robot Hardware C++ Package

High-performance C++ hardware interface for holonomic robot with STM32 microcontroller.

## Features

- **STM32 Serial Bridge**: Bidirectional communication with STM32 via UART
- **Protocol Parser**: Binary protocol with CRC-16 error detection
- **ROS2 Integration**: Publishes `/odom`, `/imu`, subscribes to `/cmd_vel`
- **TF Broadcasting**: Publishes `odom -> base_footprint` transform
- **High Performance**: <1ms latency, supports up to 2 Mbps serial

## Building

```bash
cd /path/to/ros2_ws
colcon build --packages-select robot_hardware_cpp
source install/setup.bash
```

## Running

```bash
# With default configuration
ros2 run robot_hardware_cpp stm32_bridge

# With custom parameters
ros2 run robot_hardware_cpp stm32_bridge --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p read_rate_hz:=50
```

## Configuration

Edit `config/stm32_bridge.yaml`:

```yaml
stm32_bridge:
  ros__parameters:
    serial_port: "/dev/ttyTHS0"  # Jetson hardware UART
    baudrate: 921600             # Serial baud rate
    read_rate_hz: 100            # Polling frequency
    publish_tf: true             # Publish odom TF
```

## Protocol

See `protocol/serial_protocol.md` for complete specification.

### Quick Overview

- **Message Format**: `[0xAA | MSG_ID | LENGTH | PAYLOAD | CRC16]`
- **Jetson → STM32**: Velocity commands (20 Hz), heartbeat (1 Hz)
- **STM32 → Jetson**: Odometry (100 Hz), IMU (100 Hz), motor status

## Hardware Setup

### Jetson Orin Nano Connections

- **UART**: Pin 8 (TX), Pin 10 (RX), Pin 6 (GND)
- **Device**: `/dev/ttyTHS0` (hardware UART)
- **Baud**: 921600 bps (can go up to 2 Mbps)

### STM32 Firmware

Firmware implementation is in `firmware/` directory (to be implemented separately).

## Topics

### Subscribed
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot

### Published
- `/odom` (nav_msgs/Odometry): Robot odometry from STM32
- `/imu` (sensor_msgs/Imu): IMU data from STM32

### TF Frames
- Publishes: `odom -> base_footprint`

## Troubleshooting

### Serial port permission denied
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### No data received
1. Check connections: `ls -l /dev/ttyTHS0`
2. Test with screen: `screen /dev/ttyTHS0 921600`
3. Verify STM32 is sending data
4. Check baud rate matches on both sides

### High latency
- Increase `read_rate_hz` (default 100 Hz)
- Use hardware UART (`/dev/ttyTHS0`) not USB-serial
- Reduce other system load on Jetson

## Dependencies

- rclcpp
- std_msgs
- geometry_msgs
- sensor_msgs
- nav_msgs
- tf2_ros

## License

Apache-2.0
