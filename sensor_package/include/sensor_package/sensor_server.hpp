/**
 * @file sensor_data_server.hpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <util/util_msg_data.hpp>
#include <common_srvs/SensorService.h>
#include "cloud_process.hpp"

class SensorServer
{
public:
    SensorServer(ros::NodeHandle &);

    XmlRpc::XmlRpcValue param_list;

private:
    bool service_callback(common_srvs::SensorService::Request &, common_srvs::SensorService::Response &);
    void pc_sub_callback(const sensor_msgs::PointCloud2ConstPtr &);
    void img_sub_callback(const sensor_msgs::ImageConstPtr &);
    void cam_sub_callback(const sensor_msgs::CameraInfoConstPtr &);
    void set_parameter();

    ros::NodeHandle nh_, pnh_;
    ros::Publisher sensor_pub_;
    ros::ServiceServer server_;
    ros::Subscriber pc_sub_, cam_sub_, img_sub_;
    sensor_msgs::PointCloud2 pc_data_, pc_response_data_;
    sensor_msgs::Image img_data_;
    sensor_msgs::CameraInfo cam_data_;
    CloudProcess cloud_process_;
    double LEAF_SIZE;
    std::string sensor_service_name_;
    std::string pc_pub_topic_, pc_sub_topic_, img_sub_topic_, cam_sub_topic_;
    std::string sensor_frame_, world_frame_;
    bool is_crop_;
    bool is_downsample_;
};