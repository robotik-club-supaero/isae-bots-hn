cmake_minimum_required(VERSION 3.5)

project(br_trajectories)

find_package(ament_cmake REQUIRED)

install(FILES ${CMAKE_SOURCE_DIR}/br_trajectories.cpython-312-x86_64-linux-gnu.so
        DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(
  ${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in
  ${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in
)

ament_package()