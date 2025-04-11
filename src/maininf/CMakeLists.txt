cmake_minimum_required(VERSION 3.8)
project(maininf)

# Add compiler flags for warnings (optional but recommended)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)          # For ROS 2 C++ APIs
find_package(sensor_msgs REQUIRED)     # For sensor_msgs/Image message type
find_package(OpenCV REQUIRED)          # For OpenCV functionality
find_package(cv_bridge REQUIRED)       # For converting between OpenCV and ROS image formats

# Add the executable for the camera node
add_executable(camera_node src/camera_node.cpp)

# Link the required libraries to the camera_node executable
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  ${OpenCV_LIBRARIES}
)

# Install the executable so it can be run using `ros2 run`
install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Include tests if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright checks (uncomment when copyrights are added)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint checks (uncomment when in a git repo and copyrights are added)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package
ament_package()