cmake_minimum_required(VERSION 3.8)
project(massage_motion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# 将可复用的数据类型和校验逻辑构建成库，供后续其他功能包链接。
add_library(${PROJECT_NAME}
  src/motion_types.cpp
  src/motion_validation.cpp
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
ament_export_dependencies(geometry_msgs moveit_msgs)

ament_package()
