#include <visualize_package/visualize_server.hpp>

VisualizeServiceClass::VisualizeServiceClass(): 
    pnh_("~"), cloud_counter(0)
{
    set_parameter();
    timer_ = nh_.createTimer(ros::Duration(timer_duration_), &VisualizeServiceClass::timer_callback, this);
    visualize_cloud_server_ = nh_.advertiseService(visualize_cloud_service_name_, &VisualizeServiceClass::visualize_cloud_callback, this);
}

void VisualizeServiceClass::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("visualize_service", param_list);
    timer_duration_ = param_list["timer_duration"];
    visualize_cloud_service_name_ = static_cast<std::string>(param_list["visualize_cloud_service_name"]);

}

void VisualizeServiceClass::timer_callback(const ros::TimerEvent &event)
{
    if (cloud_pub_.size() == 0) {
        return;
    }
    if (cloud_pub_.size() != pc2_multi_.size()) {
        ROS_ERROR_STREAM("cloud pub size and pc2 size is not same!!");
        return;
    }
    for (int i = 0; i < cloud_pub_.size(); i++) {
        pc2_multi_[i].header.frame_id = sensor_frame_;
        cloud_pub_[i].publish(pc2_multi_[i]);
    }
}

bool VisualizeServiceClass::visualize_cloud_callback(common_srvs::VisualizeCloudRequest &request, 
                                                common_srvs::VisualizeCloudResponse &response)
{
    cloud_pub_.resize(request.cloud_data.size());
    for (int i = 0; i < cloud_pub_.size(); i++) {
        cloud_pub_[i] = nh_.advertise<sensor_msgs::PointCloud2>(request.cloud_data[i].cloud_name + "_visualize", 10);
    }
    pc2_multi_.resize(cloud_pub_.size());
    for (int i = 0; i < request.cloud_data.size(); i++) {
        pc2_multi_[i] = util_msg_data_.cloudmsg_to_pc2_color(request.cloud_data[i]);
    }
    response.ok = true;
    return true;
}