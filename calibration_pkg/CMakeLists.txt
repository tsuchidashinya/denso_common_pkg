cmake_minimum_required(VERSION 3.0.2)
project(calibration_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf_package
  util
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calibration_pkg
#  CATKIN_DEPENDS roscpp tf_package util
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibration_server src/calibration_server.cpp src/calibration_server_node.cpp)
target_link_libraries(calibration_server
  ${catkin_LIBRARIES}
)

add_executable(calibration_client src/calibration_client.cpp src/calibration_client_node.cpp)
target_link_libraries(calibration_client
  ${catkin_LIBRARIES}
)