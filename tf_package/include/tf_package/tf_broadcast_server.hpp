#pragma once
#include <util/util_msg_data.hpp>
#include <util/util.hpp>
#include <tf_package/tf_function.hpp>
#include <common_srvs/TfBroadcastService.h>
#include <common_srvs/TfDeleteService.h>

class TfBroadcastServer
{
public:
    TfBroadcastServer(ros::NodeHandle &);
    void timer_callback(const ros::TimerEvent&);
    bool tf_broadcast_service_callback(common_srvs::TfBroadcastService::Request&, common_srvs::TfBroadcastService::Response&);
    bool tf_delete_service_callback(common_srvs::TfDeleteService::Request&, common_srvs::TfDeleteService::Response&);
    void set_parameter();
private:
    ros::NodeHandle nh_, pnh_;
    std::string tf_broadcast_service_name_, tf_delete_service_name_;
    double timer_duration_;
    TfFunction tf_function_;
    std::vector<std::string> tf_name_list_;
    std::vector<geometry_msgs::TransformStamped> tf_stamp_list_;
    XmlRpc::XmlRpcValue param_list;
    ros::Timer timer_;
    ros::ServiceServer server_, delete_server_;
};