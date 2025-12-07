cmake_minimum_required(VERSION 3.8)
project(robot_actuators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveServo.action"
  "action/ControlPump.action"
  "action/ControlGripper.action"
  DEPENDENCIES std_msgs
)

# Servo Controller Node
add_executable(servo_controller
  src/servo_controller.cpp
  src/jetson_gpio.cpp
)

target_include_directories(servo_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(servo_controller PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(servo_controller
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(servo_controller "${cpp_typesupport_target}")

# Pump Controller Node
add_executable(pump_controller
  src/pump_controller.cpp
  src/jetson_gpio.cpp
)

target_include_directories(pump_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pump_controller PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(pump_controller
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target_pump ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pump_controller "${cpp_typesupport_target_pump}")

# Install targets
install(TARGETS 
  servo_controller
  pump_controller
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
