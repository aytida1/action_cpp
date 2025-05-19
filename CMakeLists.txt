cmake_minimum_required(VERSION 3.8)
project(action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(action_template_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(count_until_action_server src/count_until_server.cpp)
ament_target_dependencies(count_until_action_server rclcpp rclcpp_action action_template_interfaces)

add_executable(count_until_action_client src/count_until_client.cpp)
ament_target_dependencies(count_until_action_client rclcpp rclcpp_action action_template_interfaces)

add_executable(move_forward_action_server src/move_forward_server.cpp)
ament_target_dependencies(move_forward_action_server rclcpp rclcpp_action action_template_interfaces geometry_msgs nav_msgs)

add_executable(move_forward_action_client src/move_forward_client.cpp)
ament_target_dependencies(move_forward_action_client rclcpp rclcpp_action action_template_interfaces)

install(TARGETS
  count_until_action_server
  count_until_action_client
  move_forward_action_client
  move_forward_action_server
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY src 
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
