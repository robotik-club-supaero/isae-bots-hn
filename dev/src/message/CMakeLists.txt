set(CMAKE_CXX_STANDARD 17)

cmake_minimum_required(VERSION 3.5)
project(message)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python) 
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(...) # Add dependencies here

################################################
## Declare ROS messages, services and actions ##
# See https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html
################################################

set(msg_files
  "msg/CircleObstacle.msg"
  "msg/DisplacementRequest.msg"
  "msg/EndOfActionMsg.msg"
  "msg/SensorObstacle.msg"
  "msg/SensorObstacleList.msg"
)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_python_install_package(message_utils)

################################################
## Ament setup ##
################################################

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()