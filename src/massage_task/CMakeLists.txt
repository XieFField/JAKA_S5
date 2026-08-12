cmake_minimum_required(VERSION 3.8)
project(massage_task)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(massage_motion REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

# demo
add_executable(minimal_task_demo src/minimal_task_demo.cpp)

ament_target_dependencies(minimal_task_demo
  rclcpp
  massage_motion
  sensor_msgs
)

target_link_libraries(minimal_task_demo
  ${PROJECT_NAME}
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

#demo install
install(
  TARGETS
  minimal_task_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
