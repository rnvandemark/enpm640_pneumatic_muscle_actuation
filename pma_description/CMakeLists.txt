cmake_minimum_required(VERSION 3.10.2)
project(pma_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
)
install(
    DIRECTORY config urdf
    DESTINATION share/${PROJECT_NAME}
)
install(
    FILES .setup_assistant
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
