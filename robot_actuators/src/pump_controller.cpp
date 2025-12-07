#include <memory>
#include <thread>
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_actuators/action/control_pump.hpp"
#include "robot_actuators/jetson_gpio.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class PumpControllerNode : public rclcpp::Node {
public:
  using ControlPump = robot_actuators::action::ControlPump;
  using GoalHandleControlPump = rclcpp_action::ServerGoalHandle<ControlPump>;

  PumpControllerNode() : Node("pump_controller") {
    // Declare parameters
    this->declare_parameter("num_pumps", 2);
    this->declare_parameter("gpio_pins", std::vector<int64_t>{79, 80});  // Example Jetson pins
    this->declare_parameter("pwm_frequency_hz", 1000);
    this->declare_parameter("max_duty_cycle", 1.0);
    this->declare_parameter("enable_pressure_sensing", false);
    
    // Get parameters
    num_pumps_ = this->get_parameter("num_pumps").as_int();
    auto gpio_pins = this->get_parameter("gpio_pins").as_integer_array();
    pwm_frequency_ = this->get_parameter("pwm_frequency_hz").as_int();
    max_duty_cycle_ = this->get_parameter("max_duty_cycle").as_double();
    
    // Initialize GPIO pins
    for (size_t i = 0; i < gpio_pins.size() && i < static_cast<size_t>(num_pumps_); i++) {
      auto gpio = std::make_unique<robot_actuators::JetsonGPIO>(gpio_pins[i]);
      
      if (!gpio->export_gpio()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to export GPIO pin %ld for pump %zu", 
                     gpio_pins[i], i);
        continue;
      }
      
      if (!gpio->set_direction(robot_actuators::GPIODirection::OUTPUT)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO pin %ld to OUTPUT", gpio_pins[i]);
        continue;
      }
      
      gpio->set_value(robot_actuators::GPIOValue::LOW);
      pump_gpios_[i] = std::move(gpio);
      pump_states_[i] = false;
      pump_duty_cycles_[i] = 0.0;
      
      RCLCPP_INFO(this->get_logger(), "Pump %zu initialized on GPIO pin %ld", i, gpio_pins[i]);
    }
    
    // Create action server
    control_pump_action_server_ = rclcpp_action::create_server<ControlPump>(
      this,
      "control_pump",
      std::bind(&PumpControllerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PumpControllerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&PumpControllerNode::handle_accepted, this, std::placeholders::_1)
    );
    
    // Create emergency stop service
    emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "emergency_stop_pumps",
      std::bind(&PumpControllerNode::handle_emergency_stop, this, 
                std::placeholders::_1, std::placeholders::_2)
    );
    
    RCLCPP_INFO(this->get_logger(), "Pump controller initialized with %d pumps", num_pumps_);
  }

  ~PumpControllerNode() {
    // Turn off all pumps on shutdown
    for (auto& [pump_id, gpio] : pump_gpios_) {
      gpio->stop_pwm();
      gpio->set_value(robot_actuators::GPIOValue::LOW);
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlPump::Goal> goal)
  {
    (void)uuid;
    
    // Validate pump ID
    if (goal->pump_id < 0 || goal->pump_id >= num_pumps_) {
      RCLCPP_WARN(this->get_logger(), "Invalid pump ID: %d", goal->pump_id);
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (pump_gpios_.find(goal->pump_id) == pump_gpios_.end()) {
      RCLCPP_WARN(this->get_logger(), "Pump %d not initialized", goal->pump_id);
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Validate duty cycle
    if (goal->duty_cycle < 0.0 || goal->duty_cycle > max_duty_cycle_) {
      RCLCPP_WARN(this->get_logger(), "Invalid duty cycle: %.2f (max: %.2f)", 
                  goal->duty_cycle, max_duty_cycle_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(this->get_logger(), "Accepting goal: Pump %d %s (duty: %.1f%%, duration: %.1fs)", 
                goal->pump_id, goal->enable ? "ON" : "OFF", 
                goal->duty_cycle * 100.0, goal->duration_sec);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleControlPump> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Received cancel request for pump %d", goal->pump_id);
    
    // Turn off pump immediately
    if (pump_gpios_.find(goal->pump_id) != pump_gpios_.end()) {
      pump_gpios_[goal->pump_id]->stop_pwm();
      pump_gpios_[goal->pump_id]->set_value(robot_actuators::GPIOValue::LOW);
      pump_states_[goal->pump_id] = false;
    }
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleControlPump> goal_handle) {
    // Start execution in a new thread
    std::thread{std::bind(&PumpControllerNode::execute_control, this, goal_handle)}.detach();
  }

  void execute_control(const std::shared_ptr<GoalHandleControlPump> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ControlPump::Feedback>();
    auto result = std::make_shared<ControlPump::Result>();
    
    int pump_id = goal->pump_id;
    bool enable = goal->enable;
    double duty_cycle = goal->duty_cycle;
    double duration_sec = goal->duration_sec;
    
    auto& gpio = pump_gpios_[pump_id];
    
    if (!enable) {
      // Turn off pump
      gpio->stop_pwm();
      gpio->set_value(robot_actuators::GPIOValue::LOW);
      pump_states_[pump_id] = false;
      pump_duty_cycles_[pump_id] = 0.0;
      
      result->success = true;
      result->total_runtime_sec = 0.0;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Pump %d turned OFF", pump_id);
      return;
    }
    
    // Turn on pump with PWM
    if (duty_cycle >= 0.99) {
      // Full power, no PWM needed
      gpio->set_value(robot_actuators::GPIOValue::HIGH);
    } else {
      gpio->set_pwm(duty_cycle, pwm_frequency_);
    }
    
    pump_states_[pump_id] = true;
    pump_duty_cycles_[pump_id] = duty_cycle;
    
    RCLCPP_INFO(this->get_logger(), "Pump %d turned ON (duty: %.1f%%)", 
                pump_id, duty_cycle * 100.0);
    
    // If duration is 0 or negative, run indefinitely
    if (duration_sec <= 0.0) {
      result->success = true;
      result->total_runtime_sec = 0.0;
      goal_handle->succeed(result);
      return;
    }
    
    // Run for specified duration with feedback
    auto start_time = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10.0);  // 10 Hz feedback
    
    while (rclcpp::ok()) {
      // Check for cancellation
      if (goal_handle->is_canceling()) {
        gpio->stop_pwm();
        gpio->set_value(robot_actuators::GPIOValue::LOW);
        pump_states_[pump_id] = false;
        
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        
        result->success = false;
        result->total_runtime_sec = elapsed;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Pump %d canceled after %.1fs", pump_id, elapsed);
        return;
      }
      
      // Calculate elapsed time
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time).count();
      
      // Check if duration complete
      if (elapsed >= duration_sec) {
        gpio->stop_pwm();
        gpio->set_value(robot_actuators::GPIOValue::LOW);
        pump_states_[pump_id] = false;
        
        result->success = true;
        result->total_runtime_sec = elapsed;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Pump %d completed %.1fs run", pump_id, elapsed);
        return;
      }
      
      // Publish feedback
      feedback->elapsed_time_sec = elapsed;
      feedback->pressure_pa = 0.0;  // Placeholder for pressure sensor
      goal_handle->publish_feedback(feedback);
      
      rate.sleep();
    }
  }

  void handle_emergency_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP: All pumps stopped");
    
    // Stop all pumps immediately
    for (auto& [pump_id, gpio] : pump_gpios_) {
      gpio->stop_pwm();
      gpio->set_value(robot_actuators::GPIOValue::LOW);
      pump_states_[pump_id] = false;
      pump_duty_cycles_[pump_id] = 0.0;
    }
    
    response->success = true;
    response->message = "All pumps emergency stopped";
  }

  // Parameters
  int num_pumps_;
  int pwm_frequency_;
  double max_duty_cycle_;
  
  // Hardware interfaces
  std::map<int, std::unique_ptr<robot_actuators::JetsonGPIO>> pump_gpios_;
  
  // State tracking
  std::map<int, bool> pump_states_;
  std::map<int, double> pump_duty_cycles_;
  
  // ROS interfaces
  rclcpp_action::Server<ControlPump>::SharedPtr control_pump_action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PumpControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
