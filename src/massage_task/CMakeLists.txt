cmake_minimum_required(VERSION 3.8)
project(massage_task)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(massage_motion REQUIRED)
find_package(rclcpp REQUIRED)


find_package(rclcpp REQUIRED)

add_executable(minimal_task_state_machine_demo
  src/minimal_task_state_machine_demo.cpp
)

target_link_libraries(minimal_task_state_machine_demo
  ${PROJECT_NAME}
)

ament_target_dependencies(minimal_task_state_machine_demo
  rclcpp
)

install(
  TARGETS minimal_task_state_machine_demo
  DESTINATION lib/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/minimal_task_state_machine.cpp
)

target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_17
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  massage_motion
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
endif()

ament_export_dependencies(massage_motion)


ament_package()
