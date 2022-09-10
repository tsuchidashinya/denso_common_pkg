/**
 * @file util_sensor.hpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include "common_header.hpp"
#include "util_base.hpp"
#include <common_msgs/CloudData.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

typedef pcl::PointXYZ PclXyz;
typedef pcl::PointXYZRGB PclRgb;

class UtilSensor
{
public:
  UtilSensor();
  static common_msgs::CloudData pcl_to_cloudmsg(pcl::PointCloud<PclXyz>);
  static pcl::PointCloud<PclXyz> cloudmsg_to_pcl(common_msgs::CloudData);
  pcl::PointCloud<PclRgb> cloudmsg_to_pclrgb(common_msgs::CloudData);
  static sensor_msgs::PointCloud2 cloudmsg_to_pc2(common_msgs::CloudData);
  sensor_msgs::PointCloud2 cloudmsg_to_pc2_color(common_msgs::CloudData);
  static common_msgs::CloudData pc2_to_cloudmsg(sensor_msgs::PointCloud2);
  static cv::Mat img_to_cv(sensor_msgs::Image, std::string);

private:
  ros::NodeHandle pnh_;
  XmlRpc::XmlRpcValue param_list;
  UtilBase util;
};