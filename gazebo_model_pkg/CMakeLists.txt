cmake_minimum_required(VERSION 3.0.2)
project(gazebo_model_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  cv_bridge
  common_msgs 
  common_srvs 
  util_package
  tf_package
  gazebo_msgs
  gazebo_dev
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES decide_object_position gazebo_model_move
 CATKIN_DEPENDS roscpp gazebo_dev
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_library(decide_object_position src/decide_object_position.cpp)
target_link_libraries(decide_object_position
  ${catkin_LIBRARIES}
)

add_library(gazebo_model_move src/gazebo_model_move.cpp)
target_link_libraries(gazebo_model_move
  ${catkin_LIBRARIES}
  decide_object_position
)

add_executable(gazebo_tf_publisher src/gazebo_tf_publisher.cpp src/gazebo_tf_publisher_node.cpp)
target_link_libraries(gazebo_tf_publisher
  ${catkin_LIBRARIES}
)

add_library(gazebo_link_visual_plugin src/gazebo_link_visual_plugin.cpp)
add_dependencies(gazebo_link_visual_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_link_visual_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS gazebo_link_visual_plugin
  #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)