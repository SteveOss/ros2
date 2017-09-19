cmake_minimum_required (VERSION 3.5)
project (iot_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (iot_msgs REQUIRED)
find_package (rosidl_default_generators REQUIRED)

add_executable (publisher_iot iot_publisher.cpp)
ament_target_dependencies (publisher_iot rclcpp iot_msgs)

install (TARGETS publisher_iot DESTINATION bin)

ament_package()
