#include "robot_hardware_cpp/serial_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

namespace robot_hardware
{

SerialInterface::SerialInterface(const std::string& port, int baudrate)
: port_(port), baudrate_(baudrate), fd_(-1)
{
}

SerialInterface::~SerialInterface()
{
  close();
}

bool SerialInterface::open()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // Save old terminal settings
  if (tcgetattr(fd_, &old_tio_) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  struct termios new_tio;
  std::memset(&new_tio, 0, sizeof(new_tio));

  // Configure serial port
  new_tio.c_cflag = CS8 | CLOCAL | CREAD;
  new_tio.c_iflag = IGNPAR;
  new_tio.c_oflag = 0;
  new_tio.c_lflag = 0;
  new_tio.c_cc[VTIME] = 0;
  new_tio.c_cc[VMIN] = 0;

  // Set baud rate
  int baud_flag = getBaudrateFlag(baudrate_);
  cfsetispeed(&new_tio, baud_flag);
  cfsetospeed(&new_tio, baud_flag);

  tcflush(fd_, TCIFLUSH);
  if (tcsetattr(fd_, TCSANOW, &new_tio) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void SerialInterface::close()
{
  if (fd_ >= 0) {
    tcsetattr(fd_, TCSANOW, &old_tio_);
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialInterface::read(uint8_t* buffer, size_t size)
{
  if (fd_ < 0) return -1;
  return ::read(fd_, buffer, size);
}

int SerialInterface::write(const uint8_t* buffer, size_t size)
{
  if (fd_ < 0) return -1;
  return ::write(fd_, buffer, size);
}

void SerialInterface::flush()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
}

int SerialInterface::getBaudrateFlag(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B921600;
  }
}

} // namespace robot_hardware
