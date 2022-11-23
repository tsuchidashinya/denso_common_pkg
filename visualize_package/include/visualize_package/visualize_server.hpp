#pragma once
#include <common_srvs/VisualizeCloud.h>
#include <common_srvs/VisualizeImage.h>
#include <util/util.hpp>
#include <util/util_msg_data.hpp>
#include <sensor_msgs/PointCloud2.h>


class VisualizeServiceClass
{
public:
    VisualizeServiceClass();
    void timer_callback(const ros::TimerEvent&);
    bool visualize_cloud_callback(common_srvs::VisualizeCloudRequest&, common_srvs::VisualizeCloudResponse&);
    bool vis_image_callback(common_srvs::VisualizeImageRequest&, common_srvs::VisualizeImageResponse&);
private:
    ros::NodeHandle nh_, pnh_;
    XmlRpc::XmlRpcValue param_list;
    std::vector<ros::Publisher> cloud_pub_, image_pub_list_;
    std::vector<sensor_msgs::PointCloud2> pc2_multi_;
    std::vector<sensor_msgs::Image> image_list_;
    std::string sensor_frame_;
    ros::Timer timer_;
    int cloud_counter;
    double timer_duration_;
    UtilMsgData util_msg_data_;
    ros::ServiceServer visualize_cloud_server_, vis_image_server_;
    std::string visualize_cloud_service_name_, vis_image_service_name_;

    void set_parameter();
};