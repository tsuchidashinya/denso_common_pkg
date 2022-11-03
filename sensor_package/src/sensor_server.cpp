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
    set_parameter();
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_cast<std::string>(param_list["pc_pub_topic"]), 10);
    pc_sub_ = nh_.subscribe(static_cast<std::string>(param_list["pc_sub_topic"]), 10, &SensorServer::pc_sub_callback, this);
    img_sub_ = nh_.subscribe(static_cast<std::string>(param_list["img_sub_topic"]), 10, &SensorServer::img_sub_callback, this);
    cam_sub_ = nh_.subscribe(static_cast<std::string>(param_list["cam_sub_topic"]), 10, &SensorServer::cam_sub_callback, this);
    server_ = nh_.advertiseService(sensor_service_name_, &SensorServer::service_callback, this);
}

/**
 * @brief pointcloud2のSubscriberのコールバック関数
 * 中でpointcloud2をpcl型へ変換している
 *
 * @param msg
 */
void SensorServer::pc_sub_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pc_data_ = *msg;
    // UtilBase::message_show("pcl_data", pcl_data_.points.size());
}

void SensorServer::set_parameter() {
    pnh_.getParam("sensor_server", param_list);
    LEAF_SIZE = param_list["leaf_size"];
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
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
    pcl::PointCloud<PclXyz> pcl_data;
    pcl::fromROSMsg(pc_data_, pcl_data);
    UtilBase::message_show("pcl_data1", pcl_data.points.size());
    pcl_data = CloudProcess::downsample_by_voxelgrid(pcl_data, LEAF_SIZE);
    UtilBase::message_show(std::to_string(LEAF_SIZE), pcl_data.points.size());
    response.cloud_data = UtilSensor::pcl_to_cloudmsg(pcl_data);
    response.image = img_data_;
    response.camera_info = cam_data_;
    return true;
}