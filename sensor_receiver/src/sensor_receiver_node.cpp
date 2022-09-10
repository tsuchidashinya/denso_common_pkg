/**
 * @file sensor_receiver_node.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <sensor_receiver/sensor_receiver.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_receiver");
    ros::NodeHandle nh;
    SensorReceiver senser_service(nh);
    ros::spin();
}