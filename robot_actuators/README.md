# Robot Actuators Package

ROS2 package for controlling robot actuators (servos, pumps, grippers) on Jetson Orin Nano.

## Features

- **Servo Control**: PCA9685 I²C PWM controller for up to 16 servos
- **Pump Control**: GPIO-based control with PWM support for variable speed
- **Action-Based Interface**: ROS2 actions for non-blocking control with feedback
- **Emergency Stop**: Service interface for immediate safety shutdown

## Hardware Requirements

### Servos
- **Controller**: PCA9685 16-channel I²C PWM controller
- **Interface**: I²C bus (default: `/dev/i2c-1`, address `0x40`)
- **Power**: External 5-6V power supply (servos draw high current)

### Pumps/Solenoids
- **Interface**: GPIO pins with MOSFET drivers
- **Power**: External 12V/24V supply through MOSFETs
- **Driver**: Logic-level MOSFETs (e.g., IRLZ44N) for 3.3V GPIO

## Installation

### Dependencies

```bash
# Install I2C tools
sudo apt-get install i2c-tools libi2c-dev

# Enable I2C on Jetson
sudo usermod -aG i2c $USER
# Reboot required
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_actuators
source install/setup.bash
```

## Usage

### Servo Controller

```bash
# Launch servo controller
ros2 run robot_actuators servo_controller --ros-args --params-file src/robot_actuators/config/servo_controller.yaml

# Send action goal (move servo 0 to 90 degrees at 30 deg/sec)
ros2 action send_goal /move_servo robot_actuators/action/MoveServo "{servo_id: 0, angle_deg: 90.0, max_speed_deg_per_sec: 30.0}"

# Emergency stop all servos
ros2 service call /emergency_stop_servos std_srvs/srv/Trigger
```

### Pump Controller

```bash
# Launch pump controller
ros2 run robot_actuators pump_controller --ros-args --params-file src/robot_actuators/config/pump_controller.yaml

# Turn on pump 0 at 80% duty cycle for 5 seconds
ros2 action send_goal /control_pump robot_actuators/action/ControlPump "{pump_id: 0, enable: true, duty_cycle: 0.8, duration_sec: 5.0}"

# Turn off pump 0
ros2 action send_goal /control_pump robot_actuators/action/ControlPump "{pump_id: 0, enable: false, duty_cycle: 0.0, duration_sec: 0.0}"

# Emergency stop all pumps
ros2 service call /emergency_stop_pumps std_srvs/srv/Trigger
```

## Action Interfaces

### MoveServo.action

**Goal:**
- `int32 servo_id`: Servo channel ID (0-15 on PCA9685)
- `float64 angle_deg`: Target angle in degrees
- `float64 max_speed_deg_per_sec`: Maximum rotation speed (0 = instant)

**Feedback:**
- `float64 current_angle`: Current servo angle
- `float64 progress_percent`: Motion progress (0-100%)

**Result:**
- `float64 final_angle`: Final achieved angle
- `bool success`: True if target reached

### ControlPump.action

**Goal:**
- `int32 pump_id`: Pump/solenoid ID
- `bool enable`: Turn on (true) or off (false)
- `float64 duty_cycle`: PWM duty cycle (0.0-1.0)
- `float64 duration_sec`: Run duration (0 = indefinite)

**Feedback:**
- `float64 elapsed_time_sec`: Time since start
- `float64 pressure_pa`: Pressure reading (if sensor available)

**Result:**
- `float64 total_runtime_sec`: Total run time
- `bool success`: True if completed normally

### ControlGripper.action

**Goal:**
- `string command`: "open", "close", "stop"
- `float64 force_newtons`: Target grip force (0 = no limit)

**Feedback:**
- `float64 position_percent`: Gripper position (0=closed, 100=open)
- `float64 current_ma`: Motor current draw

**Result:**
- `float64 final_position`: Final position
- `bool success`: True if completed

## Configuration

### Servo Configuration (`config/servo_controller.yaml`)

```yaml
servo_controller:
  ros__parameters:
    use_pca9685: true
    pca9685_i2c_bus: "/dev/i2c-1"
    pca9685_address: 0x40
    num_servos: 4
    servo_min_pulse_ms: 1.0    # 0° position
    servo_max_pulse_ms: 2.0    # 180° position
    servo_range_deg: 180.0
```

### Pump Configuration (`config/pump_controller.yaml`)

```yaml
pump_controller:
  ros__parameters:
    num_pumps: 2
    gpio_pins: [79, 80]        # Jetson GPIO pin numbers
    pwm_frequency_hz: 1000
    max_duty_cycle: 1.0
```

## Hardware Wiring

### PCA9685 I²C Connections
```
Jetson Pin 1  (3.3V)  → PCA9685 VCC
Jetson Pin 3  (SDA)   → PCA9685 SDA
Jetson Pin 5  (SCL)   → PCA9685 SCL
Jetson Pin 6  (GND)   → PCA9685 GND
External 5-6V Power   → PCA9685 V+ (servo power)
```

### GPIO Pump Connections (with MOSFET)
```
Jetson GPIO Pin → MOSFET Gate
MOSFET Source   → GND
MOSFET Drain    → Pump Negative Terminal
Pump Positive   → External Power Supply (+12V/24V)
Power Supply GND → Jetson GND (common ground)
```

## Troubleshooting

### I²C Issues
```bash
# Check I²C devices
sudo i2cdetect -y -r 1

# Should show PCA9685 at address 0x40
# If not detected, check wiring and power
```

### GPIO Permission Issues
```bash
# Add user to GPIO group
sudo usermod -aG gpio $USER
# Reboot required
```

### Servo Not Moving
- Check external power supply (5-6V, sufficient amperage)
- Verify PCA9685 initialization in logs
- Test with `i2cget` commands
- Check servo pulse width parameters match servo specs

### Pump Not Activating
- Verify GPIO pin numbers (use `gpioinfo` to check)
- Check MOSFET gate voltage (should be 3.3V when active)
- Ensure common ground between Jetson and pump power supply
- Test GPIO manually: `echo 1 > /sys/class/gpio/gpio79/value`

## Safety Notes

⚠️ **Important Safety Information**

- Always connect external power supplies for servos and pumps
- Never exceed maximum duty cycle limits
- Use emergency stop service in safety-critical applications
- Implement mechanical limit switches for actuators
- Monitor current draw to detect stalls or jams
- Add flyback diodes across inductive loads (solenoids, pumps)

## Architecture

```
┌─────────────────────────────────────────┐
│         robot_actuators Package         │
├─────────────────────────────────────────┤
│                                         │
│  ┌──────────────┐    ┌──────────────┐  │
│  │   Servo      │    │    Pump      │  │
│  │  Controller  │    │  Controller  │  │
│  └──────┬───────┘    └──────┬───────┘  │
│         │                   │          │
│         ▼                   ▼          │
│  ┌──────────────┐    ┌──────────────┐  │
│  │   PCA9685    │    │ Jetson GPIO  │  │
│  │ I²C Driver   │    │  Sysfs API   │  │
│  └──────┬───────┘    └──────┬───────┘  │
│         │                   │          │
└─────────┼───────────────────┼──────────┘
          │                   │
          ▼                   ▼
    ┌──────────┐        ┌──────────┐
    │  Servos  │        │  Pumps   │
    │ (16 max) │        │(GPIO pins)│
    └──────────┘        └──────────┘
```

## Future Enhancements

- [ ] Gripper controller implementation with force sensing
- [ ] Pressure sensor integration for pump feedback
- [ ] Current sensing for servo/motor overload detection
- [ ] Calibration routine for servo endpoints
- [ ] Trajectory planning for coordinated multi-servo motion
- [ ] CAN bus integration for industrial motor controllers

## License

Apache 2.0
