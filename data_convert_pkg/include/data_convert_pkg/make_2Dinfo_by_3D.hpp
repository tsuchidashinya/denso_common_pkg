#pragma once
#include "func_data_convertion.hpp"
#include <common_msgs/PoseData.h>
#include <tf_package/tf_basic.hpp>

class Make2DInfoBy3D
{
public:
    Make2DInfoBy3D(sensor_msgs::CameraInfo, ImageSize);
    std::vector<common_msgs::BoxPosition> get_out_data(std::vector<std::string>);
    XmlRpc::XmlRpcValue param_list;
private:
    ros::NodeHandle pnh_;
    std::vector<Point3D> get_3Dpoint_from_sensor(std::vector<std::string>);
    std::vector<common_msgs::BoxPosition> convert_3Dto2D(std::vector<Point3D>);
    std::string world_frame_, sensor_frame_;
    double object_radious_x_, object_radious_y_;
    sensor_msgs::CameraInfo cinfo_;
    ImageSize img_size_;
};