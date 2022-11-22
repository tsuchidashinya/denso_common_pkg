/**
 * @file util_msg_data.hpp
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
#include "util.hpp"
#include <common_msgs/CloudData.h>
#include <common_msgs/BoxPosition.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

typedef pcl::PointXYZ PclXyz;
typedef pcl::PointXYZRGB PclRgb;

struct YoloFormat
{
  float x;
  float y;
  float w;
  float h;
  std::string tf_name;
  std::string object_class_name;
};

class UtilMsgData
{
public:
  UtilMsgData();
  static void cloud_size_ok(common_msgs::CloudData&);
  static common_msgs::CloudData remove_ins_cloudmsg(common_msgs::CloudData, int);
  static common_msgs::CloudData concat_cloudmsg(common_msgs::CloudData, common_msgs::CloudData);
  static common_msgs::CloudData pcl_to_cloudmsg(pcl::PointCloud<PclXyz>);
  static sensor_msgs::PointCloud2 pcl_to_pc2(pcl::PointCloud<PclXyz>);
  static sensor_msgs::PointCloud2 pclrgb_to_pc2_color(pcl::PointCloud<PclRgb>);
  static pcl::PointCloud<PclRgb> pc2_color_to_pclrgb(sensor_msgs::PointCloud2);
  static pcl::PointCloud<PclXyz> pc2_to_pcl(sensor_msgs::PointCloud2);
  static pcl::PointCloud<PclXyz> cloudmsg_to_pcl(common_msgs::CloudData);
  pcl::PointCloud<PclRgb> cloudmsg_to_pclrgb(common_msgs::CloudData);
  static sensor_msgs::PointCloud2 cloudmsg_to_pc2(common_msgs::CloudData);
  sensor_msgs::PointCloud2 cloudmsg_to_pc2_color(common_msgs::CloudData);
  static common_msgs::CloudData pc2_to_cloudmsg(sensor_msgs::PointCloud2);
  static cv::Mat rosimg_to_cvimg(sensor_msgs::Image, std::string);
  static std::vector<float> caminfo_to_floatlist(sensor_msgs::CameraInfo);
  static YoloFormat pascalvoc_to_yolo(common_msgs::BoxPosition);
  static common_msgs::BoxPosition yolo_to_pascalvoc(YoloFormat, ImageSize);
  static common_msgs::BoxPosition box_position_normalized(common_msgs::BoxPosition);
private:
  ros::NodeHandle pnh_;
  XmlRpc::XmlRpcValue param_list;
  Util util;
  void set_parameter();
};