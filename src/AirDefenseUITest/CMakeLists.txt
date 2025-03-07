cmake_minimum_required(VERSION 3.5)
project(AirDefenseUITest)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add C++ executable
add_executable(frame_publisher src/frame_publisher.cpp)

# Include directories for C++
target_include_directories(frame_publisher PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# Link libraries for C++
target_link_libraries(frame_publisher
  ${OpenCV_LIBS}
)

# Declare dependencies for C++
ament_target_dependencies(frame_publisher
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install C++ executable
install(TARGETS
  frame_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python script
install(PROGRAMS
  scripts/frame_sub_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp rclpy sensor_msgs cv_bridge OpenCV)

# Finalize package
ament_package()