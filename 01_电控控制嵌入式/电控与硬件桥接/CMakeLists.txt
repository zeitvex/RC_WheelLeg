cmake_minimum_required(VERSION 3.8)
project(sim2real_hw)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sim2real_common REQUIRED)
find_package(sim2real_interfaces REQUIRED)

add_executable(sim2real_hw_node
  src/hardware_bridge_node.cpp
)

target_include_directories(sim2real_hw_node PRIVATE include)
target_compile_features(sim2real_hw_node PRIVATE cxx_std_17)
ament_target_dependencies(sim2real_hw_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  sim2real_common
  sim2real_interfaces
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS sim2real_hw_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
