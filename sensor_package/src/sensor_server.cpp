/**
 * @file sensor_package.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <sensor_package/sensor_server.hpp>

SensorServer::SensorServer(ros::NodeHandle &nh)
    : nh_(nh),
      pnh_("~")
{
    pnh_.getParam("sensor_package", param_list);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(param_list["pc_pub_topic"], 10);
    pc_sub_ = nh_.subscribe(param_list["pc_sub_topic"], 10, &SensorServer::pc_sub_callback, this);
    img_sub_ = nh_.subscribe(param_list["img_sub_topic"], 10, &SensorServer::img_sub_callback, this);
    cam_sub_ = nh_.subscribe(param_list["cam_sub_topic"], 10, &SensorServer::cam_sub_callback, this);
    server_ = nh_.advertiseService(param_list["sensor_service_name"], &SensorServer::service_callback, this);
}

/**
 * @brief pointcloud2のSubscriberのコールバック関数
 * 中でpointcloud2をpcl型へ変換している
 *
 * @param msg
 */
void SensorServer::pc_sub_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, pcl_data_);
}

/**
 * @brief camera_infoのmsgのコールバック関数
 * メンバ変数にデータを代入している
 *
 * @param msg
 */
void SensorServer::cam_sub_callback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    cam_data_ = *msg;
}

/**
 * @brief rosのimageメッセージのコールバック関数
 * メンバ変数にデータを代入している
 *
 * @param msg
 */
void SensorServer::img_sub_callback(const sensor_msgs::ImageConstPtr &msg)
{
    img_data_ = *msg;
}

bool SensorServer::service_callback(common_srvs::SensorService::Request &request, common_srvs::SensorService::Response &response)
{
    pcl_data_ = CloudProcess::downsample_by_voxelgrid(pcl_data_, std::stof(param_list["leaf_size"]));
    response.cloud_data = UtilSensor::pcl_to_cloudmsg(pcl_data_);
    response.image = img_data_;
    response.camera_info = cam_data_;
    return true;
}