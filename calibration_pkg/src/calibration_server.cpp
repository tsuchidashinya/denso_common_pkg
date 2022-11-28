#include <calibration_pkg/calibration_server.hpp>

CalibrationServer::CalibrationServer(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_paramenter();
}

void CalibrationServer::set_paramenter()
{
    pnh_.getParam("calibration_server", param_list);
    calibration_service_name_ = static_cast<std::string>(param_list["calibration_service_name"]);
    pc_sub_topic_name_ = static_cast<std::string>(param_list["pc_sub_topic_name"]);
    new_pc_topic_name_ = static_cast<std::string>(param_list["new_pc_topic_name"]);
    new_tf_frame_name_ = static_cast<std::string>(param_list["new_tf_frame_name"]);
}

bool CalibrationServer::calibration_service_callback(common_srvs::CalibrationTfMove::Request& request, common_srvs::CalibrationTfMove::Response& response)
{
    new_tf_transform_ = tf_func_.tf_listen(request.source_tf_frame, request.)
}