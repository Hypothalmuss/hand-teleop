cmake_minimum_required(VERSION 3.8)
project(hand_teleop_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
# The repo-level config/ directory is the single source of truth for every node.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
