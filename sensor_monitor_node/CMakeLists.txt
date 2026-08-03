cmake_minimum_required(VERSION 3.8)
project(sensor_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
#find_package(launch_ros REQUIRED)

add_executable(sht3x_node src/sht3x_node.cpp)
ament_target_dependencies(sht3x_node rclcpp sensor_msgs)

add_executable(ina4230_node src/ina4230_node.cpp)
ament_target_dependencies(ina4230_node rclcpp std_msgs)

add_executable(level_transmitter_node src/level_transmitter_node.cpp)
ament_target_dependencies(level_transmitter_node rclcpp std_msgs std_srvs)

add_executable(bms485_node src/bms485_node.cpp)
ament_target_dependencies(bms485_node rclcpp sensor_msgs std_srvs)

add_executable(rs485_bus_manager src/rs485_bus_manager.cpp)
ament_target_dependencies(rs485_bus_manager rclcpp std_srvs)

add_executable(rs485_coordinator src/rs485_coordinator.cpp)
ament_target_dependencies(rs485_coordinator rclcpp std_srvs)

install(TARGETS
  sht3x_node
  ina4230_node
  level_transmitter_node
  bms485_node
  rs485_bus_manager
  rs485_coordinator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()