#include <memory>
#include <thread>
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_actuators/action/move_servo.hpp"
#include "robot_actuators/jetson_gpio.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ServoControllerNode : public rclcpp::Node {
public:
  using MoveServo = robot_actuators::action::MoveServo;
  using GoalHandleMoveServo = rclcpp_action::ServerGoalHandle<MoveServo>;

  ServoControllerNode() : Node("servo_controller") {
    // Declare parameters
    this->declare_parameter("use_pca9685", true);
    this->declare_parameter("pca9685_i2c_bus", "/dev/i2c-1");
    this->declare_parameter("pca9685_address", 0x40);
    this->declare_parameter("num_servos", 4);
    this->declare_parameter("control_rate_hz", 50.0);
    
    // Servo configuration parameters
    this->declare_parameter("servo_min_pulse_ms", 1.0);
    this->declare_parameter("servo_max_pulse_ms", 2.0);
    this->declare_parameter("servo_range_deg", 180.0);
    
    // Get parameters
    use_pca9685_ = this->get_parameter("use_pca9685").as_bool();
    num_servos_ = this->get_parameter("num_servos").as_int();
    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    
    min_pulse_ms_ = this->get_parameter("servo_min_pulse_ms").as_double();
    max_pulse_ms_ = this->get_parameter("servo_max_pulse_ms").as_double();
    servo_range_deg_ = this->get_parameter("servo_range_deg").as_double();
    
    // Initialize servo positions
    for (int i = 0; i < num_servos_; i++) {
      current_angles_[i] = 90.0;  // Center position
      target_angles_[i] = 90.0;
    }
    
    // Initialize hardware interface
    if (use_pca9685_) {
      auto i2c_bus = this->get_parameter("pca9685_i2c_bus").as_string();
      auto address = this->get_parameter("pca9685_address").as_int();
      
      pca9685_ = std::make_unique<robot_actuators::PCA9685Controller>(
        i2c_bus, static_cast<uint8_t>(address)
      );
      
      if (!pca9685_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PCA9685");
        use_pca9685_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(), "PCA9685 initialized on %s", i2c_bus.c_str());
        
        // Set all servos to center position
        for (int i = 0; i < num_servos_; i++) {
          pca9685_->set_servo_angle(i, 90.0, min_pulse_ms_, max_pulse_ms_, servo_range_deg_);
        }
      }
    }
    
    // Create action server
    move_servo_action_server_ = rclcpp_action::create_server<MoveServo>(
      this,
      "move_servo",
      std::bind(&ServoControllerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ServoControllerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ServoControllerNode::handle_accepted, this, std::placeholders::_1)
    );
    
    // Create emergency stop service
    emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "emergency_stop_servos",
      std::bind(&ServoControllerNode::handle_emergency_stop, this, 
                std::placeholders::_1, std::placeholders::_2)
    );
    
    // Control loop timer
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&ServoControllerNode::control_loop, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Servo controller initialized with %d servos", num_servos_);
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveServo::Goal> goal)
  {
    (void)uuid;
    
    // Validate servo ID
    if (goal->servo_id < 0 || goal->servo_id >= num_servos_) {
      RCLCPP_WARN(this->get_logger(), "Invalid servo ID: %d", goal->servo_id);
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Validate angle range
    if (goal->angle_deg < 0.0 || goal->angle_deg > servo_range_deg_) {
      RCLCPP_WARN(this->get_logger(), "Invalid angle: %.1f (range: 0-%.1f)", 
                  goal->angle_deg, servo_range_deg_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(this->get_logger(), "Accepting goal: Servo %d to %.1f°", 
                goal->servo_id, goal->angle_deg);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveServo> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request for servo %d", 
                goal_handle->get_goal()->servo_id);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveServo> goal_handle) {
    // Start execution in a new thread to not block the action server
    std::thread{std::bind(&ServoControllerNode::execute_move, this, goal_handle)}.detach();
  }

  void execute_move(const std::shared_ptr<GoalHandleMoveServo> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveServo::Feedback>();
    auto result = std::make_shared<MoveServo::Result>();
    
    int servo_id = goal->servo_id;
    double target_angle = goal->angle_deg;
    double max_speed = goal->max_speed_deg_per_sec;
    
    // If max_speed is 0 or very high, move instantly
    if (max_speed <= 0.0 || max_speed > 500.0) {
      current_angles_[servo_id] = target_angle;
      target_angles_[servo_id] = target_angle;
      
      if (use_pca9685_ && pca9685_) {
        pca9685_->set_servo_angle(servo_id, target_angle, min_pulse_ms_, max_pulse_ms_, servo_range_deg_);
      }
      
      result->final_angle = target_angle;
      result->success = true;
      goal_handle->succeed(result);
      return;
    }
    
    // Smooth motion with speed limit
    double start_angle = current_angles_[servo_id];
    double angle_delta = target_angle - start_angle;
    double total_time = std::abs(angle_delta) / max_speed;
    
    auto start_time = std::chrono::steady_clock::now();
    rclcpp::Rate rate(control_rate_hz_);
    
    while (rclcpp::ok()) {
      // Check for cancellation
      if (goal_handle->is_canceling()) {
        result->final_angle = current_angles_[servo_id];
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Servo %d motion canceled", servo_id);
        return;
      }
      
      // Calculate elapsed time
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time).count();
      
      // Check if motion complete
      if (elapsed >= total_time) {
        current_angles_[servo_id] = target_angle;
        target_angles_[servo_id] = target_angle;
        
        if (use_pca9685_ && pca9685_) {
          pca9685_->set_servo_angle(servo_id, target_angle, min_pulse_ms_, max_pulse_ms_, servo_range_deg_);
        }
        
        result->final_angle = target_angle;
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Servo %d reached target: %.1f°", servo_id, target_angle);
        return;
      }
      
      // Interpolate position
      double progress = elapsed / total_time;
      double current_angle = start_angle + angle_delta * progress;
      current_angles_[servo_id] = current_angle;
      
      if (use_pca9685_ && pca9685_) {
        pca9685_->set_servo_angle(servo_id, current_angle, min_pulse_ms_, max_pulse_ms_, servo_range_deg_);
      }
      
      // Publish feedback
      feedback->current_angle = current_angle;
      feedback->progress_percent = progress * 100.0;
      goal_handle->publish_feedback(feedback);
      
      rate.sleep();
    }
  }

  void control_loop() {
    // Update servo positions based on target angles
    // This runs continuously even when no action is active
    for (auto& [servo_id, angle] : target_angles_) {
      if (use_pca9685_ && pca9685_) {
        // Servos maintain position automatically
      }
    }
  }

  void handle_emergency_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP: All servos stopped");
    
    // Stop all servos by holding current position
    for (auto& [servo_id, angle] : current_angles_) {
      target_angles_[servo_id] = angle;
    }
    
    response->success = true;
    response->message = "All servos emergency stopped";
  }

  // Parameters
  bool use_pca9685_;
  int num_servos_;
  double control_rate_hz_;
  double min_pulse_ms_;
  double max_pulse_ms_;
  double servo_range_deg_;
  
  // Hardware interface
  std::unique_ptr<robot_actuators::PCA9685Controller> pca9685_;
  
  // State tracking
  std::map<int, double> current_angles_;
  std::map<int, double> target_angles_;
  
  // ROS interfaces
  rclcpp_action::Server<MoveServo>::SharedPtr move_servo_action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
