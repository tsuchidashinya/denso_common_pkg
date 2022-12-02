cmake_minimum_required(VERSION 3.0.2)
project(common_srvs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    common_msgs
    sensor_msgs
)


add_service_files(
  FILES
  sensor_service/SensorService.srv
  sensor_service/SensorPC2Service.srv
  visualize_service/VisualizeCloud.srv
  visualize_service/VisualizeCloudPose.srv
  visualize_service/VisualizeImage.srv
  visualize_service/VisualizeSensorPC2.srv
  visualize_service/VisualizeCloudDelete.srv 
  network_service/NetworkCloudService.srv 
  network_service/ObjectDetectionService.srv 
  hdf5_open_service/Hdf5OpenService.srv
  hdf5_open_service/Hdf5RealPhoxiOpenService.srv 
  hdf5_open_service/Hdf5SegmentationOpenService.srv
  network_service/SemanticSegmentationService.srv 
  accuracy_service/AccuracyIouService.srv
  calibration_service/CalibrationTfMove.srv
  tf_service/TfBroadcastService.srv
  tf_service/TfDeleteService.srv
)

generate_messages(
  DEPENDENCIES
  common_msgs sensor_msgs 
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES common_srvs
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

