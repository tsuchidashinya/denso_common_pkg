#pragma once
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <tf_package/tf_function.hpp>
#include <common_srvs/CalibrationTfMove.h>
#include <common_srvs/TfBroadcastService.h>
#include <common_srvs/VisualizeSensorPC2.h>
#include <common_srvs/SensorPC2Service.h>

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
    std::string new_pc_topic_name_;
    std::string new_tf_frame_name_;
    std::string calibration_service_name_, tf_broad_service_name_, vis_sensor_pc2_service_name_, sensor_service_name_;
    ros::Timer timer_;
    TfFunction tf_func_;
    geometry_msgs::Transform new_tf_transform_;
    ros::ServiceClient tf_broad_client_, vis_cloud_client_, sensor_client_;
};