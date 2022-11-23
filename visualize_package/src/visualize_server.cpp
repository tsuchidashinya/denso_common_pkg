#include <visualize_package/visualize_server.hpp>

VisualizeServiceClass::VisualizeServiceClass(): 
    pnh_("~"), cloud_counter(0)
{
    set_parameter();
    timer_ = nh_.createTimer(ros::Duration(timer_duration_), &VisualizeServiceClass::timer_callback, this);
    visualize_cloud_server_ = nh_.advertiseService(visualize_cloud_service_name_, &VisualizeServiceClass::visualize_cloud_callback, this);
    vis_image_server_ = nh_.advertiseService(vis_image_service_name_, &VisualizeServiceClass::vis_image_callback, this);
}

void VisualizeServiceClass::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("visualize_service", param_list);
    timer_duration_ = param_list["timer_duration"];
    visualize_cloud_service_name_ = static_cast<std::string>(param_list["visualize_cloud_service_name"]);
    vis_image_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
}

void VisualizeServiceClass::timer_callback(const ros::TimerEvent &event)
{
    if (cloud_pub_.size() == 0) {
        ;
    }
    else if (cloud_pub_.size() != pc2_multi_.size()) {
        ROS_ERROR_STREAM("cloud pub size and pc2 size is not same!!");
        ;
    }
    else {
        for (int i = 0; i < cloud_pub_.size(); i++) {
            pc2_multi_[i].header.frame_id = sensor_frame_;
            cloud_pub_[i].publish(pc2_multi_[i]);
        }
    }
    if (image_pub_list_.size() == 0) {
        ;
    }
    else {
        for (int i = 0; i < image_pub_list_.size(); i++) {
            image_pub_list_[i].publish(image_list_[i]);
        }
    }

}

bool VisualizeServiceClass::vis_image_callback(common_srvs::VisualizeImageRequest &request,
                                                common_srvs::VisualizeImageResponse &response)
{
    if (request.image_list.size() != request.topic_name_list.size()) {
        ROS_ERROR_STREAM("Please image topic!!");
        return true;
    }
    image_pub_list_.resize(request.image_list.size());
    for (int i = 0; i < image_pub_list_.size(); i++) {
        image_pub_list_[i] = nh_.advertise<sensor_msgs::Image>(request.topic_name_list[i] + "_imageVisualze", 10);
    }
    image_list_.resize(image_pub_list_.size());
    for (int i = 0; i < image_list_.size(); i++) {
        image_list_[i] = request.image_list[i];
    }
    response.ok = true;
    return true;
}

bool VisualizeServiceClass::visualize_cloud_callback(common_srvs::VisualizeCloudRequest &request, 
                                                common_srvs::VisualizeCloudResponse &response)
{
    if (request.cloud_data_list.size() != request.topic_name_list.size()) {
        ROS_ERROR_STREAM("Please cloud topic!!");
        return true;
    }
    cloud_pub_.resize(request.cloud_data_list.size());
    for (int i = 0; i < cloud_pub_.size(); i++) {
        cloud_pub_[i] = nh_.advertise<sensor_msgs::PointCloud2>(request.topic_name_list[i] + "_visualize", 10);
    }
    pc2_multi_.resize(cloud_pub_.size());
    for (int i = 0; i < request.cloud_data_list.size(); i++) {
        pc2_multi_[i] = util_msg_data_.cloudmsg_to_pc2_color(request.cloud_data_list[i]);
    }
    response.ok = true;
    return true;
}