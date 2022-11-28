#include <calibration_pkg/calibration_server.hpp>

CalibrationServer::CalibrationServer(ros::NodeHandle &nh) :
nh_(nh),
pnh_("~")
{
    set_paramenter();
    tf_broad_client_ = nh_.serviceClient<common_srvs::TfBroadcastService>(tf_broad_service_name_);
    vis_cloud_client_ = nh_.serviceClient<common_srvs::VisualizeSensorPC2>(vis_sensor_pc2_service_name_);
    sensor_client_ = nh_.serviceClient<common_srvs::SensorPC2Service>(sensor_service_name_);

}

void CalibrationServer::set_paramenter()
{
    pnh_.getParam("calibration_server", param_list);
    calibration_service_name_ = static_cast<std::string>(param_list["calibration_service_name"]);
    tf_broad_service_name_ = static_cast<std::string>(param_list["tf_broadcast_service_name"]);
    vis_sensor_pc2_service_name_ = static_cast<std::string>(param_list["vis_sensor_pc2_service_name"]);
    sensor_service_name_ = static_cast<std::string>(param_list["sensor_service_name"]);
    pc_sub_topic_name_ = static_cast<std::string>(param_list["pc_sub_topic_name"]);
    new_pc_topic_name_ = static_cast<std::string>(param_list["new_pc_topic_name"]);
    new_tf_frame_name_ = static_cast<std::string>(param_list["new_tf_frame_name"]);
}

bool CalibrationServer::calibration_service_callback(common_srvs::CalibrationTfMove::Request& request, common_srvs::CalibrationTfMove::Response& response)
{
    new_tf_transform_ = tf_func_.tf_listen(request.source_tf_frame, request.keymove_tf_frame);
    geometry_msgs::TransformStamped new_tf;
    new_tf = TfFunction::make_geo_trans_stamped(new_tf_frame_name_, request.keymove_tf_frame, new_tf_transform_);
    common_srvs::TfBroadcastService tf_broadcast_srv;
    tf_broadcast_srv.request.broadcast_tf = new_tf;
    tf_broadcast_srv.request.tf_name = new_tf.child_frame_id;
    Util::client_request(tf_broad_client_, tf_broadcast_srv, tf_broad_service_name_);
    common_srvs::SensorPC2Service sensor_srv;
    sensor_srv.request.counter = 0;
    Util::client_request(sensor_client_, sensor_srv, sensor_service_name_);
    sensor_msgs::PointCloud2 pc2;
    pc2 = sensor_srv.response.pc2_data;
    pc2.header.frame_id = new_tf_frame_name_;
    common_srvs::VisualizeSensorPC2 vis_sensor_pc2_srv;
    vis_sensor_pc2_srv.request.pc2_list.push_back(pc2);
    vis_sensor_pc2_srv.request.new_topic_name_list.push_back(new_pc_topic_name_);
    Util::client_request(vis_cloud_client_, vis_sensor_pc2_srv, vis_sensor_pc2_service_name_);
    return true;
}

