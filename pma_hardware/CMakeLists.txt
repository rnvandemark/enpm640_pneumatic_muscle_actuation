cmake_minimum_required(VERSION 3.5)
project(pma_hardware)
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pma_util REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

add_library(pma_robot_interfaces SHARED
    src/interface/pma_planar_2dof_interface.cpp
)
target_include_directories(pma_robot_interfaces PRIVATE include)
ament_target_dependencies(
    pma_robot_interfaces
    pluginlib
    controller_interface
    hardware_interface
    pma_util
    rclcpp
)
pluginlib_export_plugin_description_file(
    controller_interface
    pma_hardware_plugin.xml
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS
        pma_robot_interfaces
    EXPORT export_pma_robot_interfaces
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(pma_robot_interfaces)
ament_export_targets(export_pma_robot_interfaces HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
    eigen3_cmake_module
    Eigen3
    pluginlib
    controller_interface
    hardware_interface
    pma_util
    rclcpp
)
ament_package()





