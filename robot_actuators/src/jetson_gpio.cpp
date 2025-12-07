#include "robot_actuators/jetson_gpio.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace robot_actuators {

// ============================================================================
// JetsonGPIO Implementation
// ============================================================================

JetsonGPIO::JetsonGPIO(int pin) 
  : pin_(pin), is_exported_(false), pwm_running_(false) {
}

JetsonGPIO::~JetsonGPIO() {
  stop_pwm();
  unexport_gpio();
}

std::string JetsonGPIO::gpio_path() {
  return "/sys/class/gpio/gpio" + std::to_string(pin_);
}

bool JetsonGPIO::write_file(const std::string& path, const std::string& value) {
  std::ofstream file(path);
  if (!file.is_open()) {
    return false;
  }
  file << value;
  file.close();
  return true;
}

std::string JetsonGPIO::read_file(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return "";
  }
  std::string content;
  std::getline(file, content);
  file.close();
  return content;
}

bool JetsonGPIO::export_gpio() {
  if (is_exported_) {
    return true;
  }
  
  // Check if already exported
  std::ifstream check(gpio_path() + "/value");
  if (check.good()) {
    is_exported_ = true;
    return true;
  }
  
  // Export the GPIO
  if (!write_file("/sys/class/gpio/export", std::to_string(pin_))) {
    return false;
  }
  
  // Wait for sysfs to create the files
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  is_exported_ = true;
  return true;
}

bool JetsonGPIO::unexport_gpio() {
  if (!is_exported_) {
    return true;
  }
  
  bool result = write_file("/sys/class/gpio/unexport", std::to_string(pin_));
  if (result) {
    is_exported_ = false;
  }
  return result;
}

bool JetsonGPIO::set_direction(GPIODirection dir) {
  if (!is_exported_) {
    return false;
  }
  
  std::string direction = (dir == GPIODirection::OUTPUT) ? "out" : "in";
  return write_file(gpio_path() + "/direction", direction);
}

bool JetsonGPIO::set_value(GPIOValue value) {
  if (!is_exported_) {
    return false;
  }
  
  std::string val = (value == GPIOValue::HIGH) ? "1" : "0";
  return write_file(gpio_path() + "/value", val);
}

GPIOValue JetsonGPIO::get_value() {
  if (!is_exported_) {
    return GPIOValue::LOW;
  }
  
  std::string val = read_file(gpio_path() + "/value");
  return (val == "1") ? GPIOValue::HIGH : GPIOValue::LOW;
}

void JetsonGPIO::set_pwm(double duty_cycle, int frequency_hz) {
  if (pwm_running_) {
    stop_pwm();
  }
  
  pwm_running_ = true;
  
  auto period_us = std::chrono::microseconds(1000000 / frequency_hz);
  auto on_time_us = std::chrono::microseconds(
    static_cast<long>(period_us.count() * duty_cycle)
  );
  auto off_time_us = period_us - on_time_us;
  
  pwm_thread_ = std::thread([this, on_time_us, off_time_us]() {
    while (pwm_running_) {
      if (on_time_us.count() > 0) {
        set_value(GPIOValue::HIGH);
        std::this_thread::sleep_for(on_time_us);
      }
      if (off_time_us.count() > 0) {
        set_value(GPIOValue::LOW);
        std::this_thread::sleep_for(off_time_us);
      }
    }
    set_value(GPIOValue::LOW);
  });
}

void JetsonGPIO::stop_pwm() {
  if (pwm_running_) {
    pwm_running_ = false;
    if (pwm_thread_.joinable()) {
      pwm_thread_.join();
    }
  }
}

// ============================================================================
// PCA9685Controller Implementation
// ============================================================================

// PCA9685 Registers
constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

PCA9685Controller::PCA9685Controller(const std::string& i2c_bus, uint8_t address)
  : i2c_bus_(i2c_bus), address_(address), fd_(-1), initialized_(false) {
}

PCA9685Controller::~PCA9685Controller() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool PCA9685Controller::initialize() {
  // Open I2C bus
  fd_ = open(i2c_bus_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "Failed to open I2C bus: " << i2c_bus_ << std::endl;
    return false;
  }
  
  // Set I2C slave address
  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    std::cerr << "Failed to set I2C address: 0x" << std::hex << (int)address_ << std::endl;
    close(fd_);
    fd_ = -1;
    return false;
  }
  
  // Reset PCA9685
  write_byte(PCA9685_MODE1, 0x00);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  // Set PWM frequency to 50Hz (standard servo frequency)
  set_pwm_freq(50.0);
  
  initialized_ = true;
  return true;
}

bool PCA9685Controller::write_byte(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  if (write(fd_, buffer, 2) != 2) {
    return false;
  }
  return true;
}

uint8_t PCA9685Controller::read_byte(uint8_t reg) {
  if (write(fd_, &reg, 1) != 1) {
    return 0;
  }
  uint8_t value;
  if (read(fd_, &value, 1) != 1) {
    return 0;
  }
  return value;
}

bool PCA9685Controller::set_pwm_freq(double freq_hz) {
  // Calculate prescale value: prescale = round(25MHz / (4096 * freq)) - 1
  uint8_t prescale = static_cast<uint8_t>(25000000.0 / (4096.0 * freq_hz) - 1.0 + 0.5);
  
  // Read current MODE1 register
  uint8_t old_mode = read_byte(PCA9685_MODE1);
  
  // Enter sleep mode to change prescale
  uint8_t new_mode = (old_mode & 0x7F) | 0x10;  // Sleep mode
  write_byte(PCA9685_MODE1, new_mode);
  
  // Set prescale
  write_byte(PCA9685_PRESCALE, prescale);
  
  // Restore old mode
  write_byte(PCA9685_MODE1, old_mode);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  
  // Auto-increment mode
  write_byte(PCA9685_MODE1, old_mode | 0xA0);
  
  return true;
}

bool PCA9685Controller::set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time) {
  if (channel > 15) {
    return false;
  }
  
  uint8_t reg_base = PCA9685_LED0_ON_L + 4 * channel;
  
  write_byte(reg_base, on_time & 0xFF);
  write_byte(reg_base + 1, on_time >> 8);
  write_byte(reg_base + 2, off_time & 0xFF);
  write_byte(reg_base + 3, off_time >> 8);
  
  return true;
}

bool PCA9685Controller::set_servo_angle(uint8_t channel, double angle_deg, 
                                        double min_pulse_ms, double max_pulse_ms, 
                                        double range_deg) {
  // Constrain angle to valid range
  if (angle_deg < 0) angle_deg = 0;
  if (angle_deg > range_deg) angle_deg = range_deg;
  
  // Calculate pulse width in milliseconds
  double pulse_ms = min_pulse_ms + (max_pulse_ms - min_pulse_ms) * (angle_deg / range_deg);
  
  // Convert to 12-bit value (4096 steps per 20ms at 50Hz)
  // pulse_ms / 20ms * 4096 = pulse_ms * 204.8
  uint16_t pulse_value = static_cast<uint16_t>(pulse_ms * 204.8);
  
  return set_pwm(channel, 0, pulse_value);
}

} // namespace robot_actuators
