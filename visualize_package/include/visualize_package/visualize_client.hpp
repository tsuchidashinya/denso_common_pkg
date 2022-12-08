#pragma once
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/VisualizeImage.h>
#include <common_srvs/Hdf5OpenService.h>
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
    std::string hdf5_open_service_name_, visualize_service_name_, vis_img_service_name_;
    ros::ServiceClient hdf5_client_, visualize_client_, vis_img_client_;
};