#pragma once
#include <common_srvs/VisualizeCloud.h>
#include <util/util_base.hpp>
#include <util/util_sensor.hpp>
#include <sensor_msgs/PointCloud2.h>


class VisualizeServiceClass
{
public:
    VisualizeServiceClass();
    void timer_callback(const ros::TimerEvent&);
    bool visualize_cloud_callback(common_srvs::VisualizeCloudRequest&, common_srvs::VisualizeCloudResponse&);
private:
    ros::NodeHandle nh_, pnh_;
    XmlRpc::XmlRpcValue param_list;
    std::vector<ros::Publisher> cloud_pub_;
    std::vector<sensor_msgs::PointCloud2> pc2_multi_;
    std::string sensor_frame_;
    ros::Timer timer_;
    int cloud_counter;
    double timer_duration_;
    UtilSensor util_sensor_;
    ros::ServiceServer visualize_cloud_server_;
    std::string visualize_cloud_service_name_;

    void set_parameter();
};