#include <tf_package/tf_broadcast_server.hpp>

TfBroadcastServer::TfBroadcastServer(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    server_ = nh_.advertiseService(tf_broadcast_service_name_, &TfBroadcastServer::tf_broadcast_service_callback, this);
    timer_ = nh_.createTimer(ros::Duration(timer_duration_), &TfBroadcastServer::timer_callback, this);
    delete_server_ = nh_.advertiseService(tf_delete_service_name_, &TfBroadcastServer::tf_delete_service_callback, this);
}

void TfBroadcastServer::set_parameter()
{
    pnh_.getParam("tf_broadcast_server", param_list);
    timer_duration_ = param_list["timer_duration"];
    tf_broadcast_service_name_ = static_cast<std::string>(param_list["tf_broadcast_service_name"]);
    tf_delete_service_name_ = static_cast<std::string>(param_list["tf_delete_service_name"]);
}

bool TfBroadcastServer::tf_broadcast_service_callback(common_srvs::TfBroadcastService::Request &request, 
                                                common_srvs::TfBroadcastService::Response &response)
{
    int index = Util::find_element_vector(tf_name_list_, request.tf_name);
    if (index == -1) {
        tf_name_list_.push_back(request.broadcast_tf.child_frame_id);
        tf_stamp_list_.push_back(request.broadcast_tf);
    }
    else {
        tf_name_list_[index] = request.broadcast_tf.child_frame_id;
        tf_stamp_list_[index] = request.broadcast_tf;
    }
    response.ok = true;
    return true;
}

bool TfBroadcastServer::tf_delete_service_callback(common_srvs::TfDeleteService::Request &request,
                                            common_srvs::TfDeleteService::Response &response)
{
    int index = Util::find_element_vector(tf_name_list_, request.delete_tf_name);
    if (index == -1) {
        response.is_delete_completed = 0;
    }
    else {
        tf_name_list_.erase(tf_name_list_.begin() + index);
        tf_stamp_list_.erase(tf_stamp_list_.begin() + index);
        response.is_delete_completed = 1;
    }
    return true;
}

void TfBroadcastServer::timer_callback(const ros::TimerEvent &event)
{
    if (tf_stamp_list_.size() == 0) {
        ;
    }
    else {
        for (int i = 0; i < tf_stamp_list_.size(); i++) {
            tf_function_.static_broadcast(tf_stamp_list_[i]);
        }
    }
}