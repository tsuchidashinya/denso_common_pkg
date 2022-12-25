#include <visualize_package/visualize_server.hpp>

VisualizeServiceClass::VisualizeServiceClass(): 
    pnh_("~")
{
    set_parameter();
    timer_ = nh_.createTimer(ros::Duration(timer_duration_), &VisualizeServiceClass::timer_callback, this);
    visualize_cloud_server_ = nh_.advertiseService(visualize_cloud_service_name_, &VisualizeServiceClass::visualize_cloud_callback, this);
    vis_image_server_ = nh_.advertiseService(vis_image_service_name_, &VisualizeServiceClass::vis_image_callback, this);
    vis_cloud_delete_server_ = nh_.advertiseService(vis_cloud_delete_service_name_, &VisualizeServiceClass::vis_delete_service_callback, this);
}

void VisualizeServiceClass::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("visualize_service", param_list);
    timer_duration_ = param_list["timer_duration"];
    visualize_cloud_service_name_ = static_cast<std::string>(param_list["visualize_cloud_service_name"]);
    vis_image_service_name_ = static_cast<std::string>(param_list["visualize_image_service_name"]);
    vis_cloud_delete_service_name_ = static_cast<std::string>(param_list["vis_cloud_delete_service_name"]);
}

void VisualizeServiceClass::timer_callback(const ros::TimerEvent &event)
{
    // Util::message_show("vis_pc2_list", vis_cloud_pc2_list_.size());
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
}

bool VisualizeServiceClass::vis_image_callback(common_srvs::VisualizeImageRequest &request,
                                                common_srvs::VisualizeImageResponse &response)
{
    if (request.image_list.size() != request.topic_name_list.size()) {
        ROS_ERROR_STREAM("Please image topic!!");
        return true;
    }
    for (int i = 0; i < request.topic_name_list.size(); i++) {
        int index = Util::find_element_vector(topic_image_list_, request.topic_name_list[i]);
        if (index == -1) {
            image_pub_list_.push_back(nh_.advertise<sensor_msgs::Image>(request.topic_name_list[i], 10));
            topic_image_list_.push_back(request.topic_name_list[i]);
            image_list_.push_back(request.image_list[i]);
        }
        else {
            topic_image_list_[index] = request.topic_name_list[i];
            image_list_[index] = request.image_list[i];
        }
    }
    response.ok = true;
    return true;
}

bool VisualizeServiceClass::visualize_cloud_callback(common_srvs::VisualizeCloudRequest &request, 
                                                common_srvs::VisualizeCloudResponse &response)
{
    // Util::message_show("service_come! ", "ok");
    if (request.cloud_data_list.size() != request.topic_name_list.size()) {
        ROS_ERROR_STREAM("Please cloud topic!!");
        return true;
    }
    for (int i = 0; i < request.topic_name_list.size(); i++) {
        int index = Util::find_element_vector(topic_cloud_pc2_list_, request.topic_name_list[i]);
        if (index == -1) {
            vis_cloud_pub_list_.push_back(nh_.advertise<sensor_msgs::PointCloud2>(request.topic_name_list[i], 10));
            topic_cloud_pc2_list_.push_back(request.topic_name_list[i]);
            sensor_msgs::PointCloud2 pc2 = util_msg_data_.cloudmsg_to_pc2_color(request.cloud_data_list[i]);
            if (request.cloud_data_list[i].frame_id.size() == 0) {
                pc2.header.frame_id = sensor_frame_;
            }
            else {
                pc2.header.frame_id = request.cloud_data_list[i].frame_id;
            }
            vis_cloud_pc2_list_.push_back(pc2);
        }
        else {
            vis_cloud_pc2_list_[index] = util_msg_data_.cloudmsg_to_pc2_color(request.cloud_data_list[i]);
            if (request.cloud_data_list[i].frame_id.size() == 0) {
                vis_cloud_pc2_list_[index].header.frame_id = sensor_frame_;
            }
            else {
                vis_cloud_pc2_list_[index].header.frame_id = request.cloud_data_list[i].frame_id;
            }
        }
    }
    response.ok = true;
    return true;
}

bool VisualizeServiceClass::vis_delete_service_callback(common_srvs::VisualizeDeleteServiceRequest &request,
                                                    common_srvs::VisualizeDeleteServiceResponse &response)
{
    response.delete_ok = 0;
    for (int i = 0; i < request.delete_cloud_topic_list.size(); i++) {
        int index = Util::find_element_vector(topic_cloud_pc2_list_, request.delete_cloud_topic_list[i]);
        if (index != -1) {
            topic_cloud_pc2_list_.erase(topic_cloud_pc2_list_.begin() + index);
            vis_cloud_pc2_list_.erase(vis_cloud_pc2_list_.begin() + index);
            vis_cloud_pub_list_.erase(vis_cloud_pub_list_.begin() + index);
            response.delete_ok = 1;
        }
    }
    for (int i = 0; i < request.delete_image_topic_list.size(); i++) {
        int index = Util::find_element_vector(topic_image_list_, request.delete_image_topic_list[i]);
        if (index != -1) {
            topic_image_list_.erase(topic_image_list_.begin() + index);
            image_list_.erase(image_list_.begin() + index);
            image_pub_list_.erase(image_pub_list_.begin() + index);
            response.delete_ok = 1;
        }
    }
    return true;
}