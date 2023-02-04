#include <mesh_package/mesh_cloud_server.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_cloud_node");
    ros::NodeHandle nh;
    MeshCloudServer mesh_server(nh);
    ros::spin();
}