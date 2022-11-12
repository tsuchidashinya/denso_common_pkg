#include <visualize_package/visualize_server.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_server");
    VisualizeServiceClass visualize_service;
    ros::spin();
}