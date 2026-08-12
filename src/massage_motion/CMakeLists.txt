cmake_minimum_required(VERSION 3.8)
project(massage_motion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# demo
add_executable(ptp_planning_demo
  src/ptp_planning_demo.cpp
)

add_executable(lin_planning_demo
  src/lin_planning_demo.cpp
)

add_executable(circ_planning_demo
  src/circ_planning_demo.cpp
)

add_executable(motion_planning_sdk_demo
  src/motion_planning_sdk_demo.cpp
)

add_executable(planning_execution_demo
  src/planning_execution_demo.cpp
)

target_compile_features(
  ptp_planning_demo 
  PUBLIC cxx_std_17
)

ament_target_dependencies(ptp_planning_demo
  rclcpp
)

ament_target_dependencies(lin_planning_demo
  rclcpp
)

ament_target_dependencies(circ_planning_demo
  rclcpp
)

ament_target_dependencies(motion_planning_sdk_demo
  rclcpp
)

ament_target_dependencies(planning_execution_demo
  rclcpp
)

# add library
add_library(${PROJECT_NAME}
  src/motion_types.cpp
  src/motion_validation.cpp
  src/moveit_planner_base.cpp
  src/ptp_planner.cpp
  src/lin_planner.cpp
  src/circ_planner.cpp
  src/motion_planning_sdk.cpp
  src/execution_types.cpp
  src/execution_validation.cpp
  src/moveit_trajectory_executor.cpp
  src/trajectory_endpoint_error.cpp
)

target_link_libraries(ptp_planning_demo
  ${PROJECT_NAME}
)

target_link_libraries(lin_planning_demo
  ${PROJECT_NAME}
)

target_link_libraries(circ_planning_demo
  ${PROJECT_NAME}
)

target_link_libraries(motion_planning_sdk_demo
  ${PROJECT_NAME}
)

target_link_libraries(planning_execution_demo
  ${PROJECT_NAME}
)


target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 构建阶段使用源码目录头文件，安装后使用 install 目录中的公共头文件。
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  moveit_msgs
  rclcpp
  moveit_ros_planning_interface
  rclcpp_action
  sensor_msgs
  std_msgs
)

# 安装并导出公共头文件和库目标，使其他 ament 包可以通过
# find_package(massage_motion) 使用本库。
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  moveit_msgs
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  std_msgs
)

# TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(motion_validation_test
    test/motion_validation_test.cpp
  )

  target_link_libraries(motion_validation_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(execution_validation_test
    test/execution_validation_test.cpp
  )

  target_link_libraries(execution_validation_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(trajectory_endpoint_error_test
    test/trajectory_endpoint_error_test.cpp
  )

  target_link_libraries(trajectory_endpoint_error_test
    ${PROJECT_NAME}
  )
endif()


#demo install
install(
  TARGETS 
    ptp_planning_demo
    lin_planning_demo
    circ_planning_demo
    motion_planning_sdk_demo
    planning_execution_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
