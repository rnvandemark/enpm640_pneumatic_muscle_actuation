cmake_minimum_required(VERSION 3.5)
project(pma_util)
find_package(ament_cmake REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_package()
