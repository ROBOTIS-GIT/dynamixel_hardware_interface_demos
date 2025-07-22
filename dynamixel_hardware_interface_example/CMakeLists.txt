cmake_minimum_required(VERSION 3.5)
project(dynamixel_hardware_interface_example)

find_package(ament_cmake REQUIRED)

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}/
)