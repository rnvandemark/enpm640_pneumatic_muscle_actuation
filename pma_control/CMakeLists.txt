cmake_minimum_required(VERSION 3.5)
project(pma_control)
find_package(ament_cmake REQUIRED)

find_package(angles REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pma_hardware REQUIRED)
find_package(pma_interfaces REQUIRED)
find_package(pma_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${pma_hardware_INCLUDE_DIRS}
)

add_library(sliding_mode_2dof_pressure_trajectory_controller SHARED
    src/trajectory_control/sliding_mode_2dof_pressure_trajectory_controller.cpp
)
target_include_directories(sliding_mode_2dof_pressure_trajectory_controller PRIVATE include)
ament_target_dependencies(
    sliding_mode_2dof_pressure_trajectory_controller
    angles
    builtin_interfaces
    control_msgs
    control_toolbox
    controller_interface
    eigen3_cmake_module
    Eigen3
    hardware_interface
    joint_trajectory_controller
    lifecycle_msgs
    pluginlib
    pma_hardware
    pma_interfaces
    pma_util
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    trajectory_msgs
)
pluginlib_export_plugin_description_file(
    controller_interface
    pma_control_plugin.xml
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS
        sliding_mode_2dof_pressure_trajectory_controller
    EXPORT export_sliding_mode_2dof_pressure_trajectory_controller
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(sliding_mode_2dof_pressure_trajectory_controller)
ament_export_targets(export_sliding_mode_2dof_pressure_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ament_cmake
    angles
    builtin_interfaces
    control_msgs
    control_toolbox
    controller_interface
    hardware_interface
    joint_trajectory_controller
    lifecycle_msgs
    pluginlib
    pma_hardware
    pma_interfaces
    pma_util
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    trajectory_msgs
)
ament_package()
