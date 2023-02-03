cmake_minimum_required(VERSION 3.0.2)
project(tf_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common_srvs
  roscpp
  util_package
  tf2_ros
)


catkin_python_setup()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tf_function
 CATKIN_DEPENDS common_srvs roscpp util_package tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(tf_function src/tf_function.cpp)
target_link_libraries(tf_function
  ${catkin_LIBRARIES}
)

add_executable(tf_broadcast_server src/tf_broadcast_server.cpp src/tf_broadcast_server_node.cpp)
target_link_libraries(tf_broadcast_server
  tf_function
  ${catkin_LIBRARIES}
)