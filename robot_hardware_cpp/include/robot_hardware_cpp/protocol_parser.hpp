#ifndef ROBOT_HARDWARE_CPP__PROTOCOL_PARSER_HPP_
#define ROBOT_HARDWARE_CPP__PROTOCOL_PARSER_HPP_

#include <cstdint>
#include <vector>
#include <functional>

namespace robot_hardware
{

// Message IDs
enum class MessageID : uint8_t
{
  VELOCITY_CMD = 0x10,      // Jetson -> STM32: velocity command
  ODOMETRY_FEEDBACK = 0x40, // STM32 -> Jetson: odometry data
  IMU_DATA = 0x42,          // STM32 -> Jetson: IMU data
  MOTOR_STATUS = 0x50,      // STM32 -> Jetson: motor status
  HEARTBEAT = 0xF0          // Bidirectional: keepalive
};

// Protocol constants
constexpr uint8_t START_BYTE = 0xAA;
constexpr size_t MIN_MESSAGE_SIZE = 5; // start + id + length + crc16

// Message structures
struct VelocityCommand
{
  float vx;      // m/s
  float vy;      // m/s
  float vtheta;  // rad/s
};

struct OdometryFeedback
{
  float x;       // m
  float y;       // m
  float theta;   // rad
  float vx;      // m/s
  float vy;      // m/s
  float vtheta;  // rad/s
};

struct IMUData
{
  float accel_x;  // m/s^2
  float accel_y;
  float accel_z;
  float gyro_x;   // rad/s
  float gyro_y;
  float gyro_z;
};

class ProtocolParser
{
public:
  using OdometryCallback = std::function<void(const OdometryFeedback&)>;
  using IMUCallback = std::function<void(const IMUData&)>;
  
  ProtocolParser();

  // Register callbacks
  void setOdometryCallback(OdometryCallback callback) { odom_callback_ = callback; }
  void setIMUCallback(IMUCallback callback) { imu_callback_ = callback; }

  // Parse incoming data
  void parseData(const uint8_t* data, size_t length);

  // Encode outgoing messages
  std::vector<uint8_t> encodeVelocityCommand(const VelocityCommand& cmd);
  std::vector<uint8_t> encodeHeartbeat();

private:
  std::vector<uint8_t> buffer_;
  OdometryCallback odom_callback_;
  IMUCallback imu_callback_;

  void processMessage(MessageID id, const uint8_t* payload, size_t length);
  uint16_t calculateCRC16(const uint8_t* data, size_t length);
  bool verifyCRC(const uint8_t* message, size_t length);
};

} // namespace robot_hardware

#endif // ROBOT_HARDWARE_CPP__PROTOCOL_PARSER_HPP_
