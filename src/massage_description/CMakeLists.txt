cmake_minimum_required(VERSION 3.8)
project(massage_description)

find_package(ament_cmake REQUIRED)

# 安装项目自己的机器人扩展模型和仿真测试场景。
install(
  DIRECTORY config urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
