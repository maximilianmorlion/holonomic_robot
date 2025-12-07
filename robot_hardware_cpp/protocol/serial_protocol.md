# STM32 ↔ Jetson Serial Protocol Specification

## Overview
Binary protocol for high-bandwidth, low-latency communication between Jetson Orin Nano and STM32 microcontroller.

## Message Format
```
[START_BYTE | MSG_ID | LENGTH | PAYLOAD | CRC16_LOW | CRC16_HIGH]
```

- **START_BYTE**: 0xAA (1 byte)
- **MSG_ID**: Message type identifier (1 byte)
- **LENGTH**: Payload length in bytes (1 byte)
- **PAYLOAD**: Variable length data (0-255 bytes)
- **CRC16**: CRC-16-ANSI checksum, little-endian (2 bytes)

## Message IDs

### Jetson → STM32

| ID   | Name            | Payload Size | Description                    |
|------|-----------------|--------------|--------------------------------|
| 0x10 | VELOCITY_CMD    | 12 bytes     | Holonomic velocity command     |
| 0xF0 | HEARTBEAT       | 0 bytes      | Keepalive signal               |

### STM32 → Jetson

| ID   | Name              | Payload Size | Description                  |
|------|-------------------|--------------|------------------------------|
| 0x40 | ODOMETRY_FEEDBACK | 24 bytes     | Robot pose and velocity      |
| 0x42 | IMU_DATA          | 24 bytes     | Accelerometer + gyroscope    |
| 0x50 | MOTOR_STATUS      | Variable     | Motor faults and diagnostics |
| 0xF0 | HEARTBEAT         | 0 bytes      | Keepalive acknowledgment     |

## Payload Structures

### VELOCITY_CMD (0x10)
```c
struct VelocityCommand {
  float vx;      // Linear velocity X (m/s), 4 bytes
  float vy;      // Linear velocity Y (m/s), 4 bytes
  float vtheta;  // Angular velocity (rad/s), 4 bytes
};
```

### ODOMETRY_FEEDBACK (0x40)
```c
struct OdometryFeedback {
  float x;       // Position X (m), 4 bytes
  float y;       // Position Y (m), 4 bytes
  float theta;   // Orientation (rad), 4 bytes
  float vx;      // Velocity X (m/s), 4 bytes
  float vy;      // Velocity Y (m/s), 4 bytes
  float vtheta;  // Angular velocity (rad/s), 4 bytes
};
```

### IMU_DATA (0x42)
```c
struct IMUData {
  float accel_x;  // Acceleration X (m/s²), 4 bytes
  float accel_y;  // Acceleration Y (m/s²), 4 bytes
  float accel_z;  // Acceleration Z (m/s²), 4 bytes
  float gyro_x;   // Angular velocity X (rad/s), 4 bytes
  float gyro_y;   // Angular velocity Y (rad/s), 4 bytes
  float gyro_z;   // Angular velocity Z (rad/s), 4 bytes
};
```

## CRC-16 Calculation
Uses CRC-16-ANSI (polynomial 0xA001, init 0xFFFF).

```c
uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
```

## Communication Settings
- **Baud Rate**: 921600 bps (recommended), up to 2 Mbps supported
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None (software flow control via heartbeat)

## Timing and Watchdog
- **Jetson sends**: Velocity commands at 20 Hz, heartbeat at 1 Hz
- **STM32 sends**: Odometry at 100 Hz, IMU at 100 Hz, heartbeat at 1 Hz
- **Timeout**: If no heartbeat received for 1 second, STM32 stops motors

## Error Handling
- Invalid CRC: Discard message, do not respond
- Unknown message ID: Ignore message
- Buffer overflow: Flush buffer, re-sync on next START_BYTE
