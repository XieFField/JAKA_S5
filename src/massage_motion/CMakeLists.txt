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

# demo
add_executable(ptp_planning_demo
  src/ptp_planning_demo.cpp
)

add_executable(lin_planning_demo
  src/lin_planning_demo.cpp
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

# add library
add_library(${PROJECT_NAME}
  src/motion_types.cpp
  src/motion_validation.cpp
  src/moveit_planner_base.cpp
  src/ptp_planner.cpp
  src/lin_planner.cpp
)

target_link_libraries(ptp_planning_demo
  ${PROJECT_NAME}
)

target_link_libraries(lin_planning_demo
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
)


#demo install
install(
  TARGETS 
    ptp_planning_demo
    lin_planning_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
