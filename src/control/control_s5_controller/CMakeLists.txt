cmake_minimum_required(VERSION 3.8)
project(control_s5_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_jaka_msgs REQUIRED)
find_package(control_jaka_sdk_vendor REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(s5_motion_server src/s5_motion_server.cpp)
ament_target_dependencies(s5_motion_server
  builtin_interfaces
  control_jaka_msgs
  control_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
)
target_include_directories(s5_motion_server PUBLIC
  include
  ${control_jaka_sdk_vendor_INCLUDE_DIRS}
)
target_link_libraries(s5_motion_server ${control_jaka_sdk_vendor_LIBRARY})
set_target_properties(s5_motion_server PROPERTIES INSTALL_RPATH "$ORIGIN/../..")

add_executable(s5_joint_controller src/s5_joint_controller.cpp)
ament_target_dependencies(s5_joint_controller
  control_jaka_msgs
  rclcpp
  sensor_msgs
  std_srvs
)
target_include_directories(s5_joint_controller PUBLIC include)
target_link_libraries(s5_joint_controller Qt5::Widgets)

install(TARGETS s5_motion_server s5_joint_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
