#include <calibration_pkg/calibration_client.hpp>

CalibrationClient::CalibrationClient(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_parameter();
    tf_client_ = nh_.serviceClient<common_srvs::TfBroadcastService>(tf_broad_service_name_);
    calibration_client_ = nh_.serviceClient<common_srvs::CalibrationTfMove>(calibration_service_name_);
}

void CalibrationClient::set_parameter()
{
    pnh_.getParam("common_parameter", param_list);
    world_frame_ = static_cast<std::string>(param_list["world_frame"]);
    pnh_.getParam("calibration_client", param_list);
    qxyz_step_ = param_list["qxyz_step"];
    xyz_step_ = param_list["xyz_step"];
    calibration_service_name_ = static_cast<std::string>(param_list["calibration_service_name"]);
    tf_broad_service_name_ = static_cast<std::string>(param_list["tf_broadcast_service_name"]);
    move_tf_frame_ = static_cast<std::string>(param_list["move_tf_frame"]);
}

void CalibrationClient::main()
{
    geometry_msgs::TransformStamped move_tf;
    move_tf.child_frame_id = move_tf_frame_;
    move_tf.header.frame_id = world_frame_;
    while (ros::ok()) {
        KeyBoardTf key_tf = tf_func_.get_keyboard_tf(xyz_step_, qxyz_step_);
    }
    
    
    
}