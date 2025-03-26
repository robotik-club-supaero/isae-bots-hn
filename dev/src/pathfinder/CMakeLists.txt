cmake_minimum_required(VERSION 3.5)

project(pathfinder)

find_package(ament_cmake REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

include_directories(include)
pybind11_add_module(pathfinder src/main.cpp src/Map.cpp src/ObstacleCircle.cpp src/ObstacleRect.cpp src/Point.cpp src/Segment.cpp)

install(TARGETS
  pathfinder
  DESTINATION lib/${PROJECT_NAME}
)
ament_environment_hooks(
  ${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in
  ${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in
)

ament_package()