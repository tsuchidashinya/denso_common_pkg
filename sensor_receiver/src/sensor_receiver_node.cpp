#include <sensor_receiver/sensor_receiver.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_receiver");
    ros::NodeHandle nh;
    SensorReceiver senser_service(nh);
    ros::spin();
}