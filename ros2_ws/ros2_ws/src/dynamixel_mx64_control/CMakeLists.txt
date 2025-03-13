cmake_minimum_required(VERSION 3.5)
project(dynamixel_mx64_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include Dynamixel SDK headers
include_directories(/usr/local/include/dynamixel_sdk)

# Add the executable for the Dynamixel node
add_executable(dynamixel_control_node src/dynamixel_control_node.cpp)

# Use ament_target_dependencies to set include directories, libraries, etc.
ament_target_dependencies(dynamixel_control_node rclcpp)

# Link against the Dynamixel SDK library manually
target_link_libraries(dynamixel_control_node
  /usr/local/lib/libdxl_x64_cpp.so
)

# Install the executable
install(TARGETS
  dynamixel_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
