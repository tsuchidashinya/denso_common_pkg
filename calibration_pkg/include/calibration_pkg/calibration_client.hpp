#pragma once
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <tf_package/tf_function.hpp>
#include <common_srvs/CalibrationTfMove.h>
#include <common_srvs/TfBroadcastService.h>
#include <yaml-cpp/yaml.h>

class CalibrationClient
{
public:
    CalibrationClient(ros::NodeHandle &nh);
    void main();
    void set_parameter();
private:
    double qxyz_step_, xyz_step_;
    std::string calibration_service_name_, tf_broad_service_name_;
    ros::NodeHandle nh_, pnh_;
    XmlRpc::XmlRpcValue param_list;
    TfFunction tf_func_;
    std::string new_tf_frame_name_;
    ros::ServiceClient tf_client_, calibration_client_;
    std::string world_frame_, move_tf_frame_, sensor_frame_;
    std::string write_yaml_file_path_;
};