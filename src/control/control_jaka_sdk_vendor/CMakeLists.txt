cmake_minimum_required(VERSION 3.8)
project(control_jaka_sdk_vendor)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)
install(FILES lib/libjakaAPI.so
  DESTINATION lib
)

ament_export_include_directories(include)
ament_package(CONFIG_EXTRAS "cmake/control_jaka_sdk_vendor-extras.cmake")
