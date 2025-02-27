cmake_minimum_required(VERSION 3.5)

project(pathfinder)

find_package(ament_cmake REQUIRED)

install(FILES ${CMAKE_SOURCE_DIR}/build/pathfinder.cpython-312-x86_64-linux-gnu.so
        DESTINATION lib/${PROJECT_NAME})

ament_package()