#pragma once
#include <common_srvs/SensorService.h>
#include <common_msgs/CloudData.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

class SensorReceiver
{
public:
    SensorReceiver(ros::NodeHandle&);

    XmlRpc::XmlRpcValue param_list;

private:
    bool service_callback(common_srvs::SensorService::Request&, common_srvs::SensorService::Response&);
    void pc_sub_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void img_sub_callback(const sensor_msgs::ImageConstPtr&);
    void cam_sub_callback(const sensor_msgs::CameraInfoConstPtr&);

    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_;
    ros::ServiceServer server_;
    ros::Subscriber pc_sub_, cam_sub_, img_sub_;
    common_msgs::CloudData cloud_data_;
    sensor_msgs::Image img_data_;
    sensor_msgs::CameraInfo cam_data_;
};