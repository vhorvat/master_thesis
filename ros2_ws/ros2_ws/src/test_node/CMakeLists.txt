cmake_minimum_required(VERSION 3.5)
project(test_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add the executable
add_executable(test_node src/test_node.cpp)

# Link against ROS2 libraries
ament_target_dependencies(test_node rclcpp)

# Install the executable
install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
