#pragma once
#include <util/util_msg_data.hpp>
#include <util/util.hpp>
#include <tf_package/tf_function.hpp>
#include <common_srvs/TfBroadcastService.h>

class TfBroadcastServer
{
public:
    TfBroadcastServer();
    void timer_callback(const ros::TimerEvent&);
    bool tf_broadcast_service_callback(common_srvs::TfBroadcastService::Request&, common_srvs::TfBroadcastService::Response&);
    void set_parameter();
private:
    ros::NodeHandle nh_, pnh_;
    std::string tf_broadcast_service_name_;
    double timer_duration_;
    std::vector<std::string> tf_name_list_;
    std::vector<geometry_msgs::TransformStamped> tf_stamp_list_;
};