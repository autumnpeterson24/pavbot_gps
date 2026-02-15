cmake_minimum_required(VERSION 3.8)
project(pavbot_gps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pavbot_gps src/pavbot_gps_node.cpp)

ament_target_dependencies(
  pavbot_gps
  rclcpp
  std_msgs
  sensor_msgs
)

install(
  TARGETS pavbot_gps
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
