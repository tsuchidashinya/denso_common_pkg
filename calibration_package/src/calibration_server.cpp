#include <calibration_package/calibration_server.hpp>

CalibrationServer::CalibrationServer(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_paramenter();
    tf_broad_client_ = nh_.serviceClient<common_srvs::TfBroadcastService>(tf_broad_service_name_);
    vis_cloud_client_ = nh_.serviceClient<common_srvs::VisualizeCloud>(vis_cloud_service_name_);
    sensor_client_ = nh_.serviceClient<common_srvs::SensorService>(sensor_service_name_);
    calib_server_ = nh_.advertiseService(calibration_service_name_, &CalibrationServer::calibration_service_callback, this);
}

void CalibrationServer::set_paramenter()
{
    pnh_.getParam("calibration_server", param_list);
    calibration_service_name_ = "calibration_service";
    tf_broad_service_name_ = "tf_broadcast_service";
    vis_cloud_service_name_ = "visualize_cloud_service";
    sensor_service_name_ = "sensor_service";
    new_pc_topic_name_ = static_cast<std::string>(param_list["new_pc_topic_name"]);
}

bool CalibrationServer::calibration_service_callback(common_srvs::CalibrationTfMove::Request& request, common_srvs::CalibrationTfMove::Response& response)
{
    // new_tf_transform_ = tf_func_.tf_listen(request.source_tf_frame, request.keymove_tf_frame);
    geometry_msgs::TransformStamped new_tf;
    new_tf = TfFunction::make_geo_trans_stamped(request.new_tf_frame_name, request.keymove_tf_frame, request.geo_trans);
    common_srvs::TfBroadcastService tf_broadcast_srv;
    tf_broadcast_srv.request.broadcast_tf = new_tf;
    tf_broadcast_srv.request.tf_name = new_tf.child_frame_id;
    Util::client_request(tf_broad_client_, tf_broadcast_srv, tf_broad_service_name_);
    common_srvs::SensorService sensor_srv;
    sensor_srv.request.counter = 0;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    common_msgs::CloudData cloud;
    cloud = sensor_srv.response.cloud_data;
    
    cloud.frame_id = request.new_tf_frame_name;
    common_srvs::VisualizeCloud vis_srv;
    vis_srv.request.cloud_data_list.push_back(cloud);
    vis_srv.request.topic_name_list.push_back(new_pc_topic_name_);
    Util::client_request(vis_cloud_client_, vis_srv, vis_cloud_service_name_);
    return true;
}

