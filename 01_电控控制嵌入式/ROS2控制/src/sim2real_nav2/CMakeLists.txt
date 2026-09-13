cmake_minimum_required(VERSION 3.5)
project(sim2real_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
