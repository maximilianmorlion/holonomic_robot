#ifndef ROBOT_HARDWARE_CPP__SERIAL_INTERFACE_HPP_
#define ROBOT_HARDWARE_CPP__SERIAL_INTERFACE_HPP_

#include <string>
#include <vector>
#include <functional>
#include <termios.h>

namespace robot_hardware
{

class SerialInterface
{
public:
  SerialInterface(const std::string& port, int baudrate);
  ~SerialInterface();

  bool open();
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // Non-blocking read, returns number of bytes read
  int read(uint8_t* buffer, size_t size);
  
  // Blocking write, returns number of bytes written
  int write(const uint8_t* buffer, size_t size);

  // Flush input and output buffers
  void flush();

private:
  std::string port_;
  int baudrate_;
  int fd_;
  struct termios old_tio_;

  int getBaudrateFlag(int baudrate);
};

} // namespace robot_hardware

#endif // ROBOT_HARDWARE_CPP__SERIAL_INTERFACE_HPP_
