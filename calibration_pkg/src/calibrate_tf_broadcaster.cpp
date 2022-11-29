#include <util/util.hpp>
#include <common_srvs/TfBroadcastService.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibrate_tf_broadcaster");
    ros::NodeHandle pnh("~"), nh;
    XmlRpc::XmlRpcValue param_list;
    pnh.getParam("calibrate_tf_broadcaster", param_list);
    double x, y, z, qx, qy, qz, qw;
    x = param_list["x"];
    y = param_list["y"];
    z = param_list["z"];
    qx = param_list["qx"];
    qy = param_list["qy"];
    qz = param_list["qz"];
    qw = param_list["qw"];
    pnh.getParam("common_parameter", param_list);
    std::string world_frame, sensor_frame;
    world_frame = static_cast<std::string>(param_list["world_frame"]);
    sensor_frame = static_cast<std::string>(param_list["sensor_frame"]);
    geometry_msgs::TransformStamped trans_stamp;
    trans_stamp.transform.translation.x = x;
    trans_stamp.transform.translation.y = y;
    trans_stamp.transform.translation.z = z;
    trans_stamp.transform.rotation.x = qx;
    trans_stamp.transform.rotation.y = qy;
    trans_stamp.transform.rotation.z = qz;
    trans_stamp.transform.rotation.w = qw;
    trans_stamp.child_frame_id = sensor_frame;
    trans_stamp.header.frame_id = world_frame;
    common_srvs::TfBroadcastService tf_broadcast_srv;
    tf_broadcast_srv.request.broadcast_tf = trans_stamp;
    tf_broadcast_srv.request.tf_name = sensor_frame;
    ros::ServiceClient client_;
    client_ = nh.serviceClient<common_srvs::TfBroadcastService>("tf_broadcast_service");
    Util::client_request(client_, tf_broadcast_srv, "tf_broadcast_service");
    return 0;
}