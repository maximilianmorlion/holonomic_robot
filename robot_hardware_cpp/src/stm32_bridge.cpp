#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "robot_hardware_cpp/serial_interface.hpp"
#include "robot_hardware_cpp/protocol_parser.hpp"

class STM32Bridge : public rclcpp::Node
{
public:
  STM32Bridge()
  : Node("stm32_bridge"),
    serial_(nullptr),
    parser_(),
    tf_broadcaster_(this)
  {
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS0");
    this->declare_parameter<int>("baudrate", 921600);
    this->declare_parameter<int>("read_rate_hz", 100);
    this->declare_parameter<bool>("publish_tf", true);

    // Get parameters
    std::string port = this->get_parameter("serial_port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int rate_hz = this->get_parameter("read_rate_hz").as_int();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    // Initialize serial interface
    serial_ = std::make_unique<robot_hardware::SerialInterface>(port, baudrate);
    if (!serial_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
      throw std::runtime_error("Serial port open failed");
    }

    RCLCPP_INFO(this->get_logger(), "Opened serial port: %s at %d baud", port.c_str(), baudrate);

    // Setup protocol callbacks
    parser_.setOdometryCallback(
      std::bind(&STM32Bridge::handleOdometry, this, std::placeholders::_1));
    parser_.setIMUCallback(
      std::bind(&STM32Bridge::handleIMU, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    // Create subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&STM32Bridge::cmdVelCallback, this, std::placeholders::_1));

    // Create timer for reading serial data
    auto period = std::chrono::milliseconds(1000 / rate_hz);
    read_timer_ = this->create_wall_timer(period, std::bind(&STM32Bridge::readSerial, this));

    // Heartbeat timer (1 Hz)
    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&STM32Bridge::sendHeartbeat, this));

    RCLCPP_INFO(this->get_logger(), "STM32 Bridge initialized");
  }

  ~STM32Bridge()
  {
    if (serial_) {
      serial_->close();
    }
  }

private:
  void readSerial()
  {
    uint8_t buffer[512];
    int bytes_read = serial_->read(buffer, sizeof(buffer));
    
    if (bytes_read > 0) {
      parser_.parseData(buffer, bytes_read);
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    robot_hardware::VelocityCommand cmd;
    cmd.vx = msg->linear.x;
    cmd.vy = msg->linear.y;
    cmd.vtheta = msg->angular.z;

    auto encoded = parser_.encodeVelocityCommand(cmd);
    serial_->write(encoded.data(), encoded.size());
  }

  void handleOdometry(const robot_hardware::OdometryFeedback& odom)
  {
    // Publish odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;
    odom_msg.pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    double cy = cos(odom.theta * 0.5);
    double sy = sin(odom.theta * 0.5);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;
    odom_msg.pose.pose.orientation.w = cy;

    odom_msg.twist.twist.linear.x = odom.vx;
    odom_msg.twist.twist.linear.y = odom.vy;
    odom_msg.twist.twist.angular.z = odom.vtheta;

    odom_pub_->publish(odom_msg);

    // Publish TF transform
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = this->now();
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_footprint";
      
      tf.transform.translation.x = odom.x;
      tf.transform.translation.y = odom.y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom_msg.pose.pose.orientation;
      
      tf_broadcaster_.sendTransform(tf);
    }
  }

  void handleIMU(const robot_hardware::IMUData& imu_data)
  {
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = imu_data.accel_x;
    imu_msg.linear_acceleration.y = imu_data.accel_y;
    imu_msg.linear_acceleration.z = imu_data.accel_z;

    imu_msg.angular_velocity.x = imu_data.gyro_x;
    imu_msg.angular_velocity.y = imu_data.gyro_y;
    imu_msg.angular_velocity.z = imu_data.gyro_z;

    imu_pub_->publish(imu_msg);
  }

  void sendHeartbeat()
  {
    auto heartbeat = parser_.encodeHeartbeat();
    serial_->write(heartbeat.data(), heartbeat.size());
  }

  std::unique_ptr<robot_hardware::SerialInterface> serial_;
  robot_hardware::ProtocolParser parser_;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  bool publish_tf_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<STM32Bridge>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("stm32_bridge"), "Exception: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
