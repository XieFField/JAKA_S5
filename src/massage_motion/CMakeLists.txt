cmake_minimum_required(VERSION 3.8)
project(massage_motion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# add library
add_library(${PROJECT_NAME}
  src/motion_types.cpp
  src/motion_validation.cpp
  src/moveit_planner_base.cpp
  src/ptp_planner.cpp
  src/lin_planner.cpp
  src/circ_planner.cpp
  src/motion_planning_sdk.cpp
  src/competitive_motion_planner.cpp
  src/execution_types.cpp
  src/execution_validation.cpp
  src/execution_timing.cpp
  src/trajectory_execution_contract.cpp
  src/moveit_trajectory_executor.cpp
  src/guarded_trajectory_executor.cpp
  src/joint_target_interpolation.cpp
  src/trajectory_endpoint_error.cpp
  src/relative_joint_target.cpp
  src/motion_repeatability.cpp
  src/technique_path_generator.cpp
  src/single_axis_admittance.cpp
  src/wrench_observation.cpp
  src/wrench_processor.cpp
  src/wrench_subscriber.cpp
  src/compliance_validation.cpp
  src/ros2_control_compliance_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 构建阶段使用源码目录头文件，安装后使用 install 目录中的公共头文件。
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  control_msgs
  controller_manager_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
  moveit_ros_planning_interface
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
)

# demo 源码与库实现分开存放。所有 demo 保持原可执行文件名，避免破坏已有 launch。
function(add_massage_motion_demo target)
  add_executable(${target} demo/${target}.cpp)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_link_libraries(${target} ${PROJECT_NAME})
  ament_target_dependencies(${target} rclcpp ${ARGN})
endfunction()

add_massage_motion_demo(ptp_planning_demo)
add_massage_motion_demo(lin_planning_demo)
add_massage_motion_demo(circ_planning_demo)
add_massage_motion_demo(motion_planning_sdk_demo)
add_massage_motion_demo(planning_execution_demo)
add_massage_motion_demo(admittance_response_demo geometry_msgs sensor_msgs)
add_massage_motion_demo(wrench_processing_demo geometry_msgs)
add_massage_motion_demo(force_admittance_demo geometry_msgs)
add_massage_motion_demo(compliance_backend_demo)

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
  control_msgs
  controller_manager_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  std_msgs
  trajectory_msgs
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

  ament_add_gtest(execution_timing_test
    test/execution_timing_test.cpp
  )

  target_link_libraries(execution_timing_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(trajectory_execution_contract_test
    test/trajectory_execution_contract_test.cpp
  )

  target_link_libraries(trajectory_execution_contract_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(trajectory_endpoint_error_test
    test/trajectory_endpoint_error_test.cpp
  )

  target_link_libraries(trajectory_endpoint_error_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(single_axis_admittance_test
    test/single_axis_admittance_test.cpp
  )

  target_link_libraries(single_axis_admittance_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(wrench_processor_test
    test/wrench_processor_test.cpp
  )

  target_link_libraries(wrench_processor_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(wrench_observation_test
    test/wrench_observation_test.cpp
  )

  target_link_libraries(wrench_observation_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(wrench_subscriber_test
    test/wrench_subscriber_test.cpp
  )

  target_link_libraries(wrench_subscriber_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(compliance_validation_test
    test/compliance_validation_test.cpp
  )

  target_link_libraries(compliance_validation_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(relative_joint_target_test
    test/relative_joint_target_test.cpp
  )

  target_link_libraries(relative_joint_target_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(motion_repeatability_test
    test/motion_repeatability_test.cpp
  )

  target_link_libraries(motion_repeatability_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(joint_target_interpolation_test
    test/joint_target_interpolation_test.cpp
  )

  target_link_libraries(joint_target_interpolation_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(technique_path_generator_test
    test/technique_path_generator_test.cpp
  )

  target_link_libraries(technique_path_generator_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(competitive_motion_planner_test
    test/competitive_motion_planner_test.cpp
  )

  target_link_libraries(competitive_motion_planner_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(guarded_trajectory_executor_test
    test/guarded_trajectory_executor_test.cpp
  )

  target_link_libraries(guarded_trajectory_executor_test
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
    admittance_response_demo
    wrench_processing_demo
    force_admittance_demo
    compliance_backend_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
