cmake_minimum_required(VERSION 3.0.2)
project(sensor_package)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  util
  tf_package
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES CloudProcess
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(CloudProcess src/cloud_process.cpp)

add_executable(SensorServer src/sensor_server.cpp src/sensor_server_node.cpp)


target_link_libraries(CloudProcess 
  ${catkin_LIBRARIES}
)

target_link_libraries(SensorServer
  CloudProcess
  ${catkin_LIBRARIES}
)