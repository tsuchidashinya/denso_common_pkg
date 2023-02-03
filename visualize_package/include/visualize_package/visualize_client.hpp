#pragma once
#include <util_package/util.hpp>
#include <util_package/util_msg_data.hpp>
#include <sensor_package/cloud_process.hpp>
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/VisualizeImage.h>
#include <common_srvs/Hdf5OpenAccService.h>
#include <common_srvs/Hdf5OpenSegmentationService.h>

class VisualizeClient
{
public:
    VisualizeClient(ros::NodeHandle&);
    void set_parameter();
    void main();
private:
    ros::NodeHandle nh_, pnh_;
    XmlRpc::XmlRpcValue param_list;
    std::string hdf5_open_acc_service_name_, visualize_service_name_, vis_img_service_name_;
    std::string hdf5_open_file_path_;
    ros::ServiceClient hdf5_client_, visualize_client_, vis_img_client_;
    CloudProcess cloud_process_;
    std::string world_frame_, sensor_frame_;
    bool crop_;
};