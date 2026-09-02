cmake_minimum_required(VERSION 3.8)
project(sim2real_runtime)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sim2real_common REQUIRED)
find_package(sim2real_interfaces REQUIRED)

# Search for ONNX Runtime headers and library
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS
  /usr/include
  /usr/include/onnxruntime
  /usr/local/include
  /usr/local/include/onnxruntime
  /opt/onnxruntime/include
)

find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  PATHS
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /usr/lib/aarch64-linux-gnu
  /usr/local/lib
  /opt/onnxruntime/lib
)

get_filename_component(ONNXRUNTIME_LIBRARY_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime not found! Please install it or specify include/library paths.")
endif()

add_executable(sim2real_runtime_node
  src/policy_runtime_node.cpp
)

add_executable(odom_relay_node
  src/odom_relay_node.cpp
)

target_include_directories(sim2real_runtime_node PRIVATE 
  include
  ${ONNXRUNTIME_INCLUDE_DIR}
)

target_include_directories(odom_relay_node PRIVATE include)

target_link_libraries(sim2real_runtime_node
  ${ONNXRUNTIME_LIBRARY}
)

set_target_properties(sim2real_runtime_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
)

target_compile_features(sim2real_runtime_node PRIVATE cxx_std_17)

ament_target_dependencies(sim2real_runtime_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2_ros
  sim2real_common
  sim2real_interfaces
)

ament_target_dependencies(odom_relay_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS sim2real_runtime_node odom_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/remote_uart_node.py
    src/cmd_mux_node.py
    src/web_udp_bridge_node.py
    src/simple_nav_node.py
    src/pcd_nav_click_tool.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
