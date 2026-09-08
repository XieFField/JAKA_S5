cmake_minimum_required(VERSION 3.8)
project(massage_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(massage_jaka REQUIRED)
find_package(massage_motion REQUIRED)
find_package(massage_task REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

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
  massage_jaka
  massage_motion
  massage_task
  rclcpp
  sensor_msgs
)

add_executable(r10_precontact_real src/r10_precontact_real.cpp)
target_compile_features(r10_precontact_real PUBLIC cxx_std_17)
target_include_directories(r10_precontact_real PRIVATE include)
ament_target_dependencies(r10_precontact_real
  jaka_msgs
  massage_motion
  moveit_ros_planning
  rclcpp
  sensor_msgs
)

add_executable(push_trajectory_sim src/push_trajectory_sim.cpp)
target_compile_features(push_trajectory_sim PUBLIC cxx_std_17)
target_include_directories(push_trajectory_sim PRIVATE include)
ament_target_dependencies(push_trajectory_sim
  massage_motion
  moveit_ros_planning
  rclcpp
  sensor_msgs
  yaml_cpp_vendor
)
target_link_libraries(push_trajectory_sim yaml-cpp)

add_executable(technique_trajectory_demo src/technique_trajectory_demo.cpp)
target_compile_features(technique_trajectory_demo PUBLIC cxx_std_17)
target_include_directories(technique_trajectory_demo PRIVATE include)
ament_target_dependencies(technique_trajectory_demo
  massage_motion
  moveit_ros_planning
  rclcpp
  sensor_msgs
)

add_executable(
  push_trajectory_real_plan_only
  src/push_trajectory_sim.cpp
)
target_compile_features(push_trajectory_real_plan_only PUBLIC cxx_std_17)
target_include_directories(push_trajectory_real_plan_only PRIVATE include)
target_compile_definitions(
  push_trajectory_real_plan_only
  PRIVATE MASSAGE_PLAN_ONLY_BUILD=1
)
ament_target_dependencies(push_trajectory_real_plan_only
  massage_motion
  moveit_ros_planning
  rclcpp
  sensor_msgs
  yaml_cpp_vendor
)
target_link_libraries(push_trajectory_real_plan_only yaml-cpp)

add_executable(
  home_segment_planning_real
  src/home_segment_planning_real.cpp
)
target_compile_features(home_segment_planning_real PUBLIC cxx_std_17)
target_include_directories(home_segment_planning_real PRIVATE include)
ament_target_dependencies(home_segment_planning_real
  massage_motion
  rclcpp
  sensor_msgs
)

add_executable(
  home_segment_round_trip_real
  src/home_segment_round_trip_real.cpp
)
target_compile_features(home_segment_round_trip_real PUBLIC cxx_std_17)
target_include_directories(home_segment_round_trip_real PRIVATE include)
ament_target_dependencies(home_segment_round_trip_real
  jaka_msgs
  massage_motion
  rclcpp
  sensor_msgs
)

install(
  TARGETS
    home_segment_planning_real
    home_segment_round_trip_real
    push_trajectory_sim
    technique_trajectory_demo
    push_trajectory_real_plan_only
    r10_precontact_real
    return_home_sim
    return_home_real
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件，使 ros2 launch 能从包的 share 目录中找到它们。
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
