#ifndef ROBOT_ACTUATORS_JETSON_GPIO_HPP_
#define ROBOT_ACTUATORS_JETSON_GPIO_HPP_

#include <string>
#include <fstream>
#include <chrono>
#include <thread>

namespace robot_actuators {

enum class GPIODirection {
  INPUT,
  OUTPUT
};

enum class GPIOValue {
  LOW = 0,
  HIGH = 1
};

/**
 * @brief Simple GPIO control using sysfs interface
 * Note: For production, consider using JetsonGPIO library or direct register access
 */
class JetsonGPIO {
public:
  JetsonGPIO(int pin);
  ~JetsonGPIO();

  bool export_gpio();
  bool unexport_gpio();
  bool set_direction(GPIODirection dir);
  bool set_value(GPIOValue value);
  GPIOValue get_value();
  
  // PWM simulation using software timing (for pumps/basic servo)
  void set_pwm(double duty_cycle, int frequency_hz = 1000);
  void stop_pwm();

private:
  int pin_;
  bool is_exported_;
  bool pwm_running_;
  std::thread pwm_thread_;
  
  std::string gpio_path();
  bool write_file(const std::string& path, const std::string& value);
  std::string read_file(const std::string& path);
};

/**
 * @brief I2C PCA9685 PWM controller for servo control
 * Uses I2C bus for precise 16-channel PWM generation
 */
class PCA9685Controller {
public:
  PCA9685Controller(const std::string& i2c_bus = "/dev/i2c-1", uint8_t address = 0x40);
  ~PCA9685Controller();

  bool initialize();
  bool set_pwm_freq(double freq_hz);
  bool set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time);
  bool set_servo_angle(uint8_t channel, double angle_deg, double min_pulse_ms = 1.0, 
                       double max_pulse_ms = 2.0, double range_deg = 180.0);

private:
  std::string i2c_bus_;
  uint8_t address_;
  int fd_;
  bool initialized_;
  
  bool write_byte(uint8_t reg, uint8_t value);
  uint8_t read_byte(uint8_t reg);
};

} // namespace robot_actuators

#endif // ROBOT_ACTUATORS_JETSON_GPIO_HPP_
