cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_PREFIX_PATH "/opt/ros/humble/share/ament_cmake/cmake/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()