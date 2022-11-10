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
    static std::vector<ArrayInt> write_2d_instance(std::vector<common_msgs::BoxPosition>, ImageSize);
    std::vector<common_msgs::CloudData> extract_data(common_msgs::CloudData, std::vector<ArrayInt>, ImageSize, std::vector<common_msgs::BoxPosition>);
};