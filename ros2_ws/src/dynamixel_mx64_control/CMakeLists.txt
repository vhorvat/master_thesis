cmake_minimum_required(VERSION 3.5)
project(dynamixel_mx64_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(/usr/local/include/dynamixel_sdk)
add_executable(dynamixel_control_node src/dynamixel_control_node.cpp)

ament_target_dependencies(dynamixel_control_node rclcpp std_msgs)

target_link_libraries(dynamixel_control_node
  /usr/local/lib/libdxl_x64_cpp.so
)


install(TARGETS
  dynamixel_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
