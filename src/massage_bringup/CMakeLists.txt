cmake_minimum_required(VERSION 3.8)
project(massage_bringup)

find_package(ament_cmake REQUIRED)

# 安装启动文件，使 ros2 launch 能从包的 share 目录中找到它们。
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
