cmake_minimum_required(VERSION 3.16)
project(g1_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)

add_executable(managed_bridge
  src/managed_bridge.cpp
  src/managed_bridge_main.cpp
)

target_include_directories(managed_bridge PRIVATE include)
target_link_libraries(managed_bridge PRIVATE
  rclcpp::rclcpp
  ros_gz_bridge::ros_gz_bridge
)

install(
  TARGETS managed_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rclcpp ros_gz_bridge)
ament_package()
