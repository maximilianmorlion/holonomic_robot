cmake_minimum_required(VERSION 3.8)
project(robot_hardware_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# STM32 Serial Bridge Node
add_executable(stm32_bridge
  src/stm32_bridge.cpp
  src/serial_interface.cpp
  src/protocol_parser.cpp
)

target_include_directories(stm32_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(stm32_bridge PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(stm32_bridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
)

# Install targets
install(TARGETS stm32_bridge
  DESTINATION lib/${PROJECT_NAME})

# Install config and protocol files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY protocol/
  DESTINATION share/${PROJECT_NAME}/protocol)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
