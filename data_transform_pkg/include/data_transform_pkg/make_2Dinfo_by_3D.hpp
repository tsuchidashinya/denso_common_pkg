#pragma once
#include "func_data_convertion.hpp"
#include <common_msgs/PoseData.h>
#include <tf_package/tf_basic.hpp>

class Make2DInfoBy3D
{
public:
    Make2DInfoBy3D(std::vector<float>, ImageSize);
    std::vector<common_msgs::BoxPosition> get_out_data(std::vector<std::string>);
    static cv::Mat draw_b_box(cv::Mat, std::vector<common_msgs::BoxPosition>);
    XmlRpc::XmlRpcValue param_list;
private:
    ros::NodeHandle pnh_;
    std::vector<Point3D> get_3Dpoint_from_sensor(std::vector<std::string>);
    std::vector<common_msgs::BoxPosition> convert_3Dto2D(std::vector<Point3D>, std::vector<std::string>);
    std::string world_frame_, sensor_frame_;
    double object_radious_x_, object_radious_y_;
    std::vector<float> cinfo_list_;
    ImageSize img_size_;
    TfBasic tf_basic_;
};