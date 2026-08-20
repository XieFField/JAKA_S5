cmake_minimum_required(VERSION 3.8)
project(massage_jaka)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(massage_motion REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}
  src/jaka_compliance_controller.cpp
  src/jaka_compliance_guard.cpp
  src/jaka_error.cpp
  src/jaka_lifecycle_client.cpp
  src/jaka_tool_drive_client.cpp
  src/tool_response_analysis.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  jaka_msgs
  massage_motion
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
)

add_executable(hardware_readiness_node src/hardware_readiness_node.cpp)
target_compile_features(hardware_readiness_node PUBLIC cxx_std_17)
ament_target_dependencies(hardware_readiness_node
  control_msgs
  diagnostic_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2_ros
)

add_executable(real_motion_smoke_demo demo/real_motion_smoke_demo.cpp)
target_compile_features(real_motion_smoke_demo PUBLIC cxx_std_17)
target_link_libraries(real_motion_smoke_demo ${PROJECT_NAME})
ament_target_dependencies(real_motion_smoke_demo
  massage_motion
  rclcpp
  sensor_msgs
)

add_executable(
  real_motion_repeatability_demo
  demo/real_motion_repeatability_demo.cpp
)
target_compile_features(real_motion_repeatability_demo PUBLIC cxx_std_17)
target_link_libraries(real_motion_repeatability_demo ${PROJECT_NAME})
ament_target_dependencies(real_motion_repeatability_demo
  jaka_msgs
  massage_motion
  rclcpp
  sensor_msgs
)

add_executable(real_compliance_smoke_demo demo/real_compliance_smoke_demo.cpp)
target_compile_features(real_compliance_smoke_demo PUBLIC cxx_std_17)
target_link_libraries(real_compliance_smoke_demo ${PROJECT_NAME})
ament_target_dependencies(real_compliance_smoke_demo
  rclcpp
  tf2_ros
)

add_executable(real_action_protocol_demo demo/real_action_protocol_demo.cpp)
target_compile_features(real_action_protocol_demo PUBLIC cxx_std_17)
ament_target_dependencies(real_action_protocol_demo
  control_msgs
  jaka_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
)

add_executable(real_ft_observation_demo demo/real_ft_observation_demo.cpp)
target_compile_features(real_ft_observation_demo PUBLIC cxx_std_17)
ament_target_dependencies(real_ft_observation_demo
  geometry_msgs
  massage_motion
  rclcpp
)

add_executable(real_tool_drive_smoke_demo
  demo/real_tool_drive_smoke_demo.cpp
)
target_compile_features(real_tool_drive_smoke_demo PUBLIC cxx_std_17)
target_link_libraries(real_tool_drive_smoke_demo ${PROJECT_NAME})
ament_target_dependencies(real_tool_drive_smoke_demo
  jaka_msgs
  massage_motion
  rclcpp
  sensor_msgs
  tf2_ros
)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS
    hardware_readiness_node
    real_action_protocol_demo
    real_compliance_smoke_demo
    real_ft_observation_demo
    real_motion_repeatability_demo
    real_motion_smoke_demo
    real_tool_drive_smoke_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  jaka_msgs
  massage_motion
  rclcpp
  std_srvs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(jaka_error_test test/jaka_error_test.cpp)
  target_link_libraries(jaka_error_test ${PROJECT_NAME})

  ament_add_gtest(
    jaka_compliance_guard_test
    test/jaka_compliance_guard_test.cpp
  )
  target_link_libraries(jaka_compliance_guard_test ${PROJECT_NAME})

  ament_add_gtest(
    jaka_compliance_controller_test
    test/jaka_compliance_controller_test.cpp
  )
  target_link_libraries(jaka_compliance_controller_test ${PROJECT_NAME})
  ament_target_dependencies(jaka_compliance_controller_test
    geometry_msgs
    jaka_msgs
    rclcpp
    sensor_msgs
    std_srvs
    tf2_ros
  )

  ament_add_gtest(
    tool_response_analysis_test
    test/tool_response_analysis_test.cpp
  )
  target_link_libraries(tool_response_analysis_test ${PROJECT_NAME})

  ament_add_gtest(
    jaka_tool_drive_client_test
    test/jaka_tool_drive_client_test.cpp
  )
  target_link_libraries(jaka_tool_drive_client_test ${PROJECT_NAME})
  ament_target_dependencies(jaka_tool_drive_client_test
    jaka_msgs
    rclcpp
    std_srvs
  )
endif()

ament_package()
