cmake_minimum_required (VERSION 3.5)
project (iot_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake REQUIRED)
find_package (rosidl_default_generators REQUIRED)

set (msg_files msg/NVP.msg msg/Instance.msg)

rosidl_generate_interfaces (${PROJECT_NAME} ${msg_files})

install (DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}/)

ament_package()
