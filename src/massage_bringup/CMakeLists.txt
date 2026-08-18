cmake_minimum_required(VERSION 3.8)
project(massage_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(massage_motion REQUIRED)
find_package(massage_task REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(return_home_sim src/return_home_sim.cpp)
target_compile_features(return_home_sim PUBLIC cxx_std_17)
target_include_directories(return_home_sim PRIVATE include)
ament_target_dependencies(return_home_sim
  massage_motion
  massage_task
  rclcpp
  sensor_msgs
)

add_executable(return_home_real src/return_home_real.cpp)
target_compile_features(return_home_real PUBLIC cxx_std_17)
target_include_directories(return_home_real PRIVATE include)
ament_target_dependencies(return_home_real
  jaka_msgs
  massage_motion
  massage_task
  rclcpp
  sensor_msgs
)

install(
  TARGETS return_home_sim return_home_real
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件，使 ros2 launch 能从包的 share 目录中找到它们。
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
