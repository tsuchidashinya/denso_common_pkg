#pragma once
#include "func_data_convertion.hpp"

class Get3DBy2D
{
public:
    Get3DBy2D(sensor_msgs::CameraInfo, ImageSize);
    std::vector<common_msgs::CloudData> get_out_data(common_msgs::CloudData, std::vector<common_msgs::BoxPosition>);
    
    
private:
    sensor_msgs::CameraInfo cinfo_;
    ImageSize im_size_;
    std::vector<ArrayInt> write_2d_instance_multi(std::vector<common_msgs::BoxPosition>);
    std::vector<ArrayInt> write_2d_instance(common_msgs::BoxPosition);
    std::vector<common_msgs::CloudData> extract_data_multi(common_msgs::CloudData, std::vector<ArrayInt>, std::vector<common_msgs::BoxPosition>);
    common_msgs::CloudData extract_data(common_msgs::CloudData, std::vector<ArrayInt>, common_msgs::BoxPosition);
};