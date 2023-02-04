#include <calibration_package/calibration_client.hpp>

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
    sensor_frame_ = static_cast<std::string>(param_list["sensor_frame"]);
    pnh_.getParam("calibration_client", param_list);
    qxyz_step_ = param_list["qxyz_step"];
    xyz_step_ = param_list["xyz_step"];
    calibration_service_name_ = "calibration_service";
    tf_broad_service_name_ = "tf_broadcast_service";
    move_tf_frame_ = static_cast<std::string>(param_list["move_tf_frame"]);
    new_tf_frame_name_ = static_cast<std::string>(param_list["new_tf_frame_name"]);
    write_yaml_file_path_ = static_cast<std::string>(param_list["write_yaml_file_path"]);
    Util::message_show("yaml_file", write_yaml_file_path_);
}

void CalibrationClient::main()
{
    geometry_msgs::TransformStamped move_tf;
    move_tf.transform.rotation = TfFunction::tf2_quat_to_geo_quat(TfFunction::rotate_xyz_make(0, 0, 0));
    move_tf.child_frame_id = move_tf_frame_;
    move_tf.header.frame_id = world_frame_;
    
    while (ros::ok()) {
        KeyBoardTf key_tf = tf_func_.get_keyboard_tf(xyz_step_, qxyz_step_);
        move_tf.transform = tf_func_.add_keyboard_tf(move_tf.transform, key_tf);
        common_srvs::TfBroadcastService tf_broad_srv;
        tf_broad_srv.request.broadcast_tf = move_tf;
        tf_broad_srv.request.tf_name = move_tf_frame_;
        Util::client_request(tf_client_, tf_broad_srv, tf_broad_service_name_);
        if (key_tf.quit) {
            break;
        }
    }
    geometry_msgs::Transform new_tf_transform;
    new_tf_transform = tf_func_.tf_listen(sensor_frame_, move_tf_frame_);
    while (ros::ok()) {
        KeyBoardTf key_tf = tf_func_.get_keyboard_tf(xyz_step_, qxyz_step_);
        move_tf.transform = tf_func_.add_keyboard_tf(move_tf.transform, key_tf);
        common_srvs::TfBroadcastService tf_broad_srv;
        tf_broad_srv.request.broadcast_tf = move_tf;
        tf_broad_srv.request.tf_name = move_tf_frame_;
        Util::client_request(tf_client_, tf_broad_srv, tf_broad_service_name_);
        common_srvs::CalibrationTfMove calib_tf_move_srv;
        calib_tf_move_srv.request.keymove_tf_frame =move_tf_frame_;
        calib_tf_move_srv.request.new_tf_frame_name = new_tf_frame_name_;
        calib_tf_move_srv.request.geo_trans = new_tf_transform;
        Util::client_request(calibration_client_, calib_tf_move_srv, calibration_service_name_);
        if (key_tf.quit) {
            break;
        }
    }
    geometry_msgs::Transform final_tf = tf_func_.tf_listen(new_tf_frame_name_, world_frame_);
    YAML::Node coodinate;
    coodinate["calibrate_tf_broadcaster"]["x"] = final_tf.translation.x;
    coodinate["calibrate_tf_broadcaster"]["y"] = final_tf.translation.y;
    coodinate["calibrate_tf_broadcaster"]["z"] = final_tf.translation.z;
    coodinate["calibrate_tf_broadcaster"]["qx"] = final_tf.rotation.x;
    coodinate["calibrate_tf_broadcaster"]["qy"] = final_tf.rotation.y;
    coodinate["calibrate_tf_broadcaster"]["qz"] = final_tf.rotation.z;
    coodinate["calibrate_tf_broadcaster"]["qw"] = final_tf.rotation.w;
    std::ofstream file(write_yaml_file_path_, std::ios::out);
    YAML::Emitter out;
    out << coodinate;
    file << out.c_str();
}

