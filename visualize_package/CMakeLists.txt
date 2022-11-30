cmake_minimum_required(VERSION 3.0.2)
project(visualize_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common_msgs
  common_srvs
  cv_bridge
  pcl_ros
  roscpp
  tf_package
  util
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visualize_package
#  CATKIN_DEPENDS common_msgs common_srvs cv_bridge pcl_ros roscpp tf_package util
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(visualize_server src/visualize_server.cpp src/visualize_server_node.cpp)
target_link_libraries(visualize_server
  ${catkin_LIBRARIES}
)

add_executable(visualize_segmentation_client src/visualize_segmentation_client.cpp src/visualize_client_node.cpp)
target_link_libraries(visualize_segmentation_client
  ${catkin_LIBRARIES}
)

add_executable(visualize_acc_client src/visualize_acc_client.cpp src/visualize_client_node.cpp)
target_link_libraries(visualize_acc_client
  ${catkin_LIBRARIES}
)