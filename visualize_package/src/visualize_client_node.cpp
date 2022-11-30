#include <visualize_package/visualize_client.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "viualize_client");
    ros::NodeHandle nh;
    ros::Duration(0.5).sleep();
    VisualizeClient visualize_client(nh);
    visualize_client.main();
    return 0;
}