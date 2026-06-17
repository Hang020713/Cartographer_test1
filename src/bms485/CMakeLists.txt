cmake_minimum_required(VERSION 3.8)
project(bms485_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsStatus.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# The node executable
add_executable(bms485_node src/bms485_node.cpp)
ament_target_dependencies(bms485_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Link the node against the generated message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bms485_node "${cpp_typesupport_target}")

install(TARGETS bms485_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()