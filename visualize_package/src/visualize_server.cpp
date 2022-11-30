#include <visualize_package/visualize_server.hpp>

VisualizeServiceClass::VisualizeServiceClass(): 
    pnh_("~"), cloud_counter(0)
{
    set_parameter();
    timer_ = nh_.createTimer(ros::Duration(timer_duration_), &VisualizeServiceClass::timer_callback, this);
    visualize_cloud_server_ = nh_.advertiseService(visualize_cloud_service_name_, &VisualizeServiceClass::visualize_cloud_callback, this);
    vis_image_server_ = nh_.advertiseService(vis_image_service_name_, &VisualizeServiceClass::vis_image_callback, this);
    vis_sensor_server_ = nh_.advertiseService(vis_sensor_pc2_service_name_, &VisualizeServiceClass::vis_sensor_pc2_callback, this);
}

void VisualizeServiceClass::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("visualize_service", param_list);
    timer_duration_ = param_list["timer_duration"];
    visualize_cloud_service_name_ = static_cast<std::string>(param_list["visualize_cloud_service_name"]);
    vis_image_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
    vis_sensor_pc2_service_name_ = static_cast<std::string>(param_list["vis_sensor_pc2_service_name"]);
}

void VisualizeServiceClass::timer_callback(const ros::TimerEvent &event)
{
    if (vis_cloud_pub_list_.size() == 0) {
        ;
    }
    else if (vis_cloud_pub_list_.size() != vis_cloud_pc2_list_.size()) {
        ROS_ERROR_STREAM("cloud pub size and pc2 size is not same!!");
        ;
    }
    else {
        for (int i = 0; i < vis_cloud_pub_list_.size(); i++) {
            vis_cloud_pc2_list_[i].header.stamp = ros::Time::now();
            vis_cloud_pub_list_[i].publish(vis_cloud_pc2_list_[i]);
        }
    }
    if (image_pub_list_.size() == 0) {
        ;
    }
    else {
        for (int i = 0; i < image_pub_list_.size(); i++) {
            image_list_[i].header.stamp = ros::Time::now();
            image_pub_list_[i].publish(image_list_[i]);
        }
    }
    if (vis_sensor_pc2_list_.size() == 0) {
        ;
    }
    else {
        for (int i = 0; i < vis_sensor_pc2_pub_list_.size(); i++) {
            vis_sensor_pc2_list_[i].header.stamp = ros::Time::now();
            vis_sensor_pc2_pub_list_[i].publish(vis_sensor_pc2_list_[i]);
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
    Util::message_show("visualize_cloud", "ok");
    if (request.cloud_data_list.size() != request.topic_name_list.size()) {
        ROS_ERROR_STREAM("Please cloud topic!!");
        return true;
    }
    Util::message_show("cloud_data_size", request.cloud_data_list.size());
    vis_cloud_pub_list_.resize(request.cloud_data_list.size());
    for (int i = 0; i < vis_cloud_pub_list_.size(); i++) {
        vis_cloud_pub_list_[i] = nh_.advertise<sensor_msgs::PointCloud2>(request.topic_name_list[i] + "_visualize", 10);
    }
    vis_cloud_pc2_list_.resize(vis_cloud_pub_list_.size());
    for (int i = 0; i < request.cloud_data_list.size(); i++) {
        vis_cloud_pc2_list_[i] = util_msg_data_.cloudmsg_to_pc2_color(request.cloud_data_list[i]);
        vis_cloud_pc2_list_[i].header.frame_id = sensor_frame_;
    }
    response.ok = true;
    return true;
}

bool VisualizeServiceClass::vis_sensor_pc2_callback(common_srvs::VisualizeSensorPC2Request &request,
                                                common_srvs::VisualizeSensorPC2Response &response)
{
    if (request.new_topic_name_list.size() != request.pc2_list.size()) {
        ROS_ERROR_STREAM("Please cloud topic!!");
        return true;
    }
    vis_sensor_pc2_pub_list_.resize(request.pc2_list.size());
    for (int i = 0; i < vis_sensor_pc2_pub_list_.size(); i++) {
        vis_sensor_pc2_pub_list_[i] = nh_.advertise<sensor_msgs::PointCloud2>(request.new_topic_name_list[i], 10);
    }
    vis_sensor_pc2_list_.resize(request.pc2_list.size());
    vis_sensor_pc2_list_ = request.pc2_list;
    response.ok = true;
    return true;
}