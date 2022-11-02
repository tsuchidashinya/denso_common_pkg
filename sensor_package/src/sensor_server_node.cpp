/**
 * @file sensor_data_server_node.cpp
 * @author tsuchidashinya (tsuchida.shinya413@mail.kyutech.jp)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <sensor_package/sensor_server.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_package");
    ros::NodeHandle nh;
    SensorServer senser_service(nh);
    ros::spin();
}