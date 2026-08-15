cmake_minimum_required(VERSION 3.8)
project(massage_task)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(massage_motion REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/minimal_task_state_machine.cpp
  src/press_task_state_machine.cpp
)

target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_17
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  massage_motion
)

# demo 源码只负责装配和验收，状态机实现保留在 src/ 库目标中。
function(add_massage_task_demo target)
  add_executable(${target} demo/${target}.cpp)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_link_libraries(${target} ${PROJECT_NAME})
  ament_target_dependencies(${target} rclcpp massage_motion ${ARGN})
endfunction()

add_massage_task_demo(minimal_task_demo sensor_msgs)
add_massage_task_demo(compliance_contact_demo)
add_massage_task_demo(
  compliant_press_task_demo
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
)

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

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  geometry_msgs
  massage_motion
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(minimal_task_state_machine_test
    test/minimal_task_state_machine_test.cpp
  )

  target_link_libraries(minimal_task_state_machine_test
    ${PROJECT_NAME}
  )

  ament_add_gtest(press_task_state_machine_test
    test/press_task_state_machine_test.cpp
  )

  target_link_libraries(press_task_state_machine_test
    ${PROJECT_NAME}
  )
endif()

#demo install
install(
  TARGETS
  minimal_task_demo
  compliance_contact_demo
  compliant_press_task_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
