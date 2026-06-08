################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_hardware_interface_demos)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

set(DYNAMIXEL_SDK_CPP_DIR "" CACHE PATH "Path to the DynamixelSDK/c++ directory")
if(NOT DYNAMIXEL_SDK_CPP_DIR)
  foreach(candidate
      "${CMAKE_CURRENT_SOURCE_DIR}/../libs/DynamixelSDK/c++"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../DynamixelSDK/c++")
    if(EXISTS "${candidate}/include/dynamixel_sdk/dynamixel_sdk.h")
      set(DYNAMIXEL_SDK_CPP_DIR "${candidate}")
      break()
    endif()
  endforeach()
endif()

if(NOT EXISTS "${DYNAMIXEL_SDK_CPP_DIR}/include/dynamixel_sdk/dynamixel_sdk.h")
  message(FATAL_ERROR
    "DynamixelSDK/c++ was not found. Set DYNAMIXEL_SDK_CPP_DIR or mount "
    "DynamixelSDK at /root/ros2_ws/src/DynamixelSDK.")
endif()

set(DYNAMIXEL_SDK_E2D2_SOURCES
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/packet_handler.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/protocol1_packet_handler.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/protocol2_packet_handler.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/group_handler.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/group_sync_read.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/group_bulk_read.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/group_fast_sync_read.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/group_fast_bulk_read.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/port_handler.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/port_handler_linux.cpp"
  "${DYNAMIXEL_SDK_CPP_DIR}/src/dynamixel_sdk/port_handler_e2d2.cpp"
)

add_library(dynamixel_sdk_e2d2_cpp STATIC ${DYNAMIXEL_SDK_E2D2_SOURCES})
target_include_directories(dynamixel_sdk_e2d2_cpp
  PUBLIC
    "${DYNAMIXEL_SDK_CPP_DIR}/include"
    "${DYNAMIXEL_SDK_CPP_DIR}/include/dynamixel_sdk"
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(dynamixel_sdk_e2d2_cpp
    PRIVATE
      -Wno-unused-parameter
      -Wno-unused-variable
      -Wno-unused-but-set-variable
  )
endif()

add_executable(e2d2_read_latency_test
  src/e2d2_read_latency_test.cpp
)
target_link_libraries(e2d2_read_latency_test
  dynamixel_sdk_e2d2_cpp
  Threads::Threads
)

install(TARGETS
  e2d2_read_latency_test
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
