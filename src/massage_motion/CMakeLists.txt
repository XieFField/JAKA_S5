cmake_minimum_required(VERSION 3.8)
project(massage_motion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

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
  src/pose_ik_candidate_generator.cpp
  src/pose_ik_competitive_planner.cpp
  src/link_trajectory_geometry.cpp
  src/elbow_posture_geometry.cpp
  src/progressive_joint_checkpoints.cpp
  src/cartesian_path_verification.cpp
  src/execution_types.cpp
  src/execution_validation.cpp
  src/execution_timing.cpp
  src/trajectory_execution_contract.cpp
  src/native_ptp_contract.cpp
  src/moveit_trajectory_executor.cpp
  src/hybrid_trajectory_executor.cpp
  src/guarded_trajectory_executor.cpp
  src/joint_target_interpolation.cpp
  src/trajectory_endpoint_error.cpp
  src/relative_joint_target.cpp
  src/motion_repeatability.cpp
  src/technique_path_generator.cpp
  src/tool_orientation.cpp
  src/single_axis_admittance.cpp
  src/wrench_observation.cpp
  src/wrench_processor.cpp
  src/wrench_frame_transform.cpp
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
  moveit_core
  moveit_msgs
  rclcpp
  moveit_ros_planning_interface
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
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
add_massage_motion_demo(
  push_pose_tuner
  geometry_msgs
  interactive_markers
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  yaml_cpp_vendor
)
target_link_libraries(push_pose_tuner yaml-cpp)
add_massage_motion_demo(
  push_pose_tuner_keyboard
  geometry_msgs
  std_srvs
)
add_massage_motion_demo(
  wrench_frame_transformer
  geometry_msgs
  tf2
  tf2_ros
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
  control_msgs
  controller_manager_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  trajectory_msgs
)

# TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(urdf REQUIRED)

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

  ament_add_gtest(native_ptp_contract_test
    test/native_ptp_contract_test.cpp
  )
  target_link_libraries(native_ptp_contract_test ${PROJECT_NAME})

  ament_add_gtest(hybrid_trajectory_executor_test
    test/hybrid_trajectory_executor_test.cpp
  )
  target_link_libraries(hybrid_trajectory_executor_test ${PROJECT_NAME})

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

  ament_add_gtest(wrench_frame_transform_test
    test/wrench_frame_transform_test.cpp
  )

  target_link_libraries(wrench_frame_transform_test
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

  ament_add_gtest(tool_orientation_test
    test/tool_orientation_test.cpp
  )

  target_link_libraries(tool_orientation_test
    ${PROJECT_NAME}
  )

  ament_target_dependencies(tool_orientation_test
    srdfdom
    urdf
  )

  ament_add_gtest(competitive_motion_planner_test
    test/competitive_motion_planner_test.cpp
  )

  target_link_libraries(competitive_motion_planner_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(pose_ik_candidate_generator_test
    test/pose_ik_candidate_generator_test.cpp
  )

  target_link_libraries(pose_ik_candidate_generator_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(pose_ik_competitive_planner_test
    test/pose_ik_competitive_planner_test.cpp
  )

  target_link_libraries(pose_ik_competitive_planner_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(link_trajectory_geometry_test
    test/link_trajectory_geometry_test.cpp
  )

  target_link_libraries(link_trajectory_geometry_test
    ${PROJECT_NAME}
  )

  ament_target_dependencies(link_trajectory_geometry_test
    srdfdom
    urdf
  )

  ament_add_gtest(elbow_posture_geometry_test
    test/elbow_posture_geometry_test.cpp
  )

  target_link_libraries(elbow_posture_geometry_test
    ${PROJECT_NAME}
  )

  ament_target_dependencies(elbow_posture_geometry_test
    srdfdom
    urdf
  )

  ament_add_gtest(progressive_joint_checkpoints_test
    test/progressive_joint_checkpoints_test.cpp
  )

  target_link_libraries(progressive_joint_checkpoints_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(cartesian_path_verification_test
    test/cartesian_path_verification_test.cpp
  )

  target_link_libraries(cartesian_path_verification_test
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
    push_pose_tuner
    push_pose_tuner_keyboard
    wrench_frame_transformer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
