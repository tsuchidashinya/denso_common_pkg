#pragma once
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <tf_package/tf_function.hpp>
#include <common_srvs/CalibrationTfMove.h>

class CalibrationClient
{
public:
    CalibrationClient(ros::NodeHandle nh);
    void main();
    void set_parameter();
private:
    double qxyz_step_, xyz_step_;
    std::string calibration_service_name_;
};