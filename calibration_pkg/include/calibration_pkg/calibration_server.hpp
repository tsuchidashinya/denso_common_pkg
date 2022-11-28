#pragma once
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <tf_package/tf_basic.hpp>
#include <common_srvs/CalibrationTfMove.h>

class CalibrationServer
{
public:
    CalibrationServer(ros::NodeHandle&);
    void set_paramenter();
    bool calibration_service_callback(common_srvs::CalibrationTfMove::Request&, common_srvs::CalibrationTfMove::Response&);
    void pc_sub_callback(sensor_msgs::PointCloud2ConstPtr&);
    void timer_callback(const ros::TimerEvent&);
private:
    ros::NodeHandle nh_, pnh_;
    XmlRpc::XmlRpcValue param_list;
    std::string pc_sub_topic_name_, new_pc_topic_name_;
    std::string new_tf_frame_name_;
    std::string calibration_service_name_;
    ros::Timer timer_;
    TfFunction tf_func_;
    geometry_msgs::Transform new_tf_transform_;
};