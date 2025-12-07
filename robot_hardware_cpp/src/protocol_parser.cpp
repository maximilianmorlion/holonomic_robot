#include "robot_hardware_cpp/protocol_parser.hpp"
#include <cstring>
#include <algorithm>

namespace robot_hardware
{

ProtocolParser::ProtocolParser()
{
  buffer_.reserve(1024);
}

void ProtocolParser::parseData(const uint8_t* data, size_t length)
{
  // Append to buffer
  buffer_.insert(buffer_.end(), data, data + length);

  // Process complete messages
  while (buffer_.size() >= MIN_MESSAGE_SIZE) {
    // Find start byte
    auto start_it = std::find(buffer_.begin(), buffer_.end(), START_BYTE);
    if (start_it == buffer_.end()) {
      buffer_.clear();
      break;
    }

    // Remove bytes before start
    if (start_it != buffer_.begin()) {
      buffer_.erase(buffer_.begin(), start_it);
    }

    // Check if we have enough data for header
    if (buffer_.size() < 3) break;

    uint8_t msg_id = buffer_[1];
    uint8_t msg_len = buffer_[2];
    size_t total_len = 3 + msg_len + 2; // start + id + len + payload + crc16

    // Wait for complete message
    if (buffer_.size() < total_len) break;

    // Verify CRC
    if (!verifyCRC(buffer_.data(), total_len)) {
      // Bad CRC, discard this message
      buffer_.erase(buffer_.begin());
      continue;
    }

    // Process message
    const uint8_t* payload = buffer_.data() + 3;
    processMessage(static_cast<MessageID>(msg_id), payload, msg_len);

    // Remove processed message
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);
  }
}

void ProtocolParser::processMessage(MessageID id, const uint8_t* payload, size_t length)
{
  switch (id) {
    case MessageID::ODOMETRY_FEEDBACK:
      if (length == sizeof(OdometryFeedback) && odom_callback_) {
        OdometryFeedback odom;
        std::memcpy(&odom, payload, sizeof(OdometryFeedback));
        odom_callback_(odom);
      }
      break;

    case MessageID::IMU_DATA:
      if (length == sizeof(IMUData) && imu_callback_) {
        IMUData imu;
        std::memcpy(&imu, payload, sizeof(IMUData));
        imu_callback_(imu);
      }
      break;

    default:
      // Unknown message, ignore
      break;
  }
}

std::vector<uint8_t> ProtocolParser::encodeVelocityCommand(const VelocityCommand& cmd)
{
  std::vector<uint8_t> message;
  message.reserve(3 + sizeof(VelocityCommand) + 2);

  message.push_back(START_BYTE);
  message.push_back(static_cast<uint8_t>(MessageID::VELOCITY_CMD));
  message.push_back(sizeof(VelocityCommand));

  const uint8_t* cmd_bytes = reinterpret_cast<const uint8_t*>(&cmd);
  message.insert(message.end(), cmd_bytes, cmd_bytes + sizeof(VelocityCommand));

  uint16_t crc = calculateCRC16(message.data(), message.size());
  message.push_back(crc & 0xFF);
  message.push_back((crc >> 8) & 0xFF);

  return message;
}

std::vector<uint8_t> ProtocolParser::encodeHeartbeat()
{
  std::vector<uint8_t> message = {START_BYTE, static_cast<uint8_t>(MessageID::HEARTBEAT), 0};
  uint16_t crc = calculateCRC16(message.data(), message.size());
  message.push_back(crc & 0xFF);
  message.push_back((crc >> 8) & 0xFF);
  return message;
}

uint16_t ProtocolParser::calculateCRC16(const uint8_t* data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= static_cast<uint16_t>(data[i]);
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

bool ProtocolParser::verifyCRC(const uint8_t* message, size_t length)
{
  if (length < 2) return false;
  
  uint16_t received_crc = message[length - 2] | (message[length - 1] << 8);
  uint16_t calculated_crc = calculateCRC16(message, length - 2);
  
  return received_crc == calculated_crc;
}

} // namespace robot_hardware
