cmake_minimum_required(VERSION 3.8)
project(odometry_bridge_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(odometry_bridge src/odometry_bridge.cpp)

# Specify dependencies
ament_target_dependencies(odometry_bridge
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Install executable
install(TARGETS
  odometry_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
