#include <tf_package/tf_broadcast_server.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcast_server");
    ros::NodeHandle nh;
    TfBroadcastServer tf_bro_server(nh);
    ros::spin();
    return 0;
}