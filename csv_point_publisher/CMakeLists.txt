cmake_minimum_required(VERSION 3.5)
project(csv_point_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(csv_point_publisher_node src/csv_point_publisher_node.cpp)

# Link against dependencies
ament_target_dependencies(csv_point_publisher_node
  rclcpp
  visualization_msgs
)

# Install executables and configuration files
install(TARGETS csv_point_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package macro
ament_package()
